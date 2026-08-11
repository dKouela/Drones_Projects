"""Software-In-The-Loop simulator lifecycle.

Two backends are available:

``stable`` (default)
    Current ArduCopter (4.x), downloaded from ArduPilot's own firmware server.
    These are the Cygwin builds Mission Planner ships, so they run natively on
    Windows with no WSL or toolchain needed.

``legacy``
    ArduCopter 3.3 via the ``dronekit-sitl`` package. Kept because it is a
    single dependency with no download of its own beyond its binary, but the
    firmware dates from 2015 and ignores several modern MAVLink commands.

Both expose the same interface, so :mod:`drone.control` neither knows nor cares
which one is flying.
"""

import base64
import logging
import os
import platform
import socket
import subprocess
import time
import urllib.request
from pathlib import Path

log = logging.getLogger(__name__)

# ArduPilot's default simulated home: CMAC airfield, Canberra.
DEFAULT_HOME = (-35.363261, 149.165230, 584, 353)

# Mission Planner's SITL builds, tracking the current stable release.
FIRMWARE_BASE = "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable"
BINARY_NAME = "ArduCopter.elf"

# The Cygwin runtime the Windows build links against.
CYGWIN_RUNTIME = (
    "cygatomic-1.dll", "cyggcc_s-1.dll", "cyggcc_s-seh-1.dll", "cyggomp-1.dll",
    "cygiconv-2.dll", "cygintl-8.dll", "cygquadmath-0.dll", "cygssp-0.dll",
    "cygstdc++-6.dll", "cygwin1.dll",
)

CACHE_DIR = Path.home() / ".ardupilot_sitl" / "copter"

# --- WSL backend -----------------------------------------------------------

WSL_DISTRO = "Ubuntu"
# Official Linux SITL build, same source commit as the Windows one.
WSL_BINARY_URL = (
    "https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter"
)
# ArduPilot's own SITL parameter file, used verbatim.
COPTER_PARM_URL = (
    "https://raw.githubusercontent.com/ArduPilot/ardupilot/master/"
    "Tools/autotest/default_params/copter.parm"
)
WSL_HOME_DIR = "$HOME/ardupilot_sitl"
# Preferred: the checked-out ArduPilot tree. Falls back to a direct download.
WSL_REPO_PARM = "$HOME/ardupilot/Tools/autotest/default_params/copter.parm"
# A source build, which is what `./waf copter` produces. Preferred over the
# published binary: see the note in WslSimulator about glibc compatibility.
WSL_SOURCE_BINARY = "$HOME/ardupilot/build/sitl/bin/arducopter"

# A stock ArduCopter 4.x parameter set will not arm: no frame is configured
# ("Motors: Check frame class and type") and the simulated IMUs read as
# uncalibrated ("3D Accel calibration needed"). ArduPilot's sim_vehicle.py
# solves this with Tools/autotest/default_params/copter.parm; this is a copy of
# the parts that matter here.
#
# The INS_ACC3 values are deliberately zero while INS_ACC1/2 are not. SITL
# simulates two accelerometers, and ArduPilot rejects the calibration outright
# if more instances look calibrated than actually exist -- so giving the
# non-existent third one non-zero offsets breaks arming rather than helping it.
DEFAULT_PARAMS = """\
FRAME_CLASS     1
FRAME_TYPE      0

FS_THR_ENABLE   1
BATT_MONITOR    4
FENCE_RADIUS    150

ATC_RAT_YAW_P   0.3
ATC_RAT_YAW_I   0.02
MOT_THST_EXPO   0.65
MOT_THST_HOVER  0.39
MOT_BAT_VOLT_MIN 9.6
MOT_BAT_VOLT_MAX 12.8

COMPASS_OFS_X   5
COMPASS_OFS_Y   13
COMPASS_OFS_Z   -18
COMPASS_OFS2_X  5
COMPASS_OFS2_Y  13
COMPASS_OFS2_Z  -18
COMPASS_OFS3_X  5
COMPASS_OFS3_Y  13
COMPASS_OFS3_Z  -18

# Small non-zero offsets are what mark the INS as calibrated.
INS_ACCOFFS_X   0.001
INS_ACCOFFS_Y   0.001
INS_ACCOFFS_Z   0.001
INS_ACCSCAL_X   1.001
INS_ACCSCAL_Y   1.001
INS_ACCSCAL_Z   1.001
INS_ACC2OFFS_X  0.001
INS_ACC2OFFS_Y  0.001
INS_ACC2OFFS_Z  0.001
INS_ACC2SCAL_X  1.001
INS_ACC2SCAL_Y  1.001
INS_ACC2SCAL_Z  1.001
INS_ACC3OFFS_X  0.000
INS_ACC3OFFS_Y  0.000
INS_ACC3OFFS_Z  0.000
INS_ACC3SCAL_X  1.000
INS_ACC3SCAL_Y  1.000
INS_ACC3SCAL_Z  1.000

RC1_MIN         1000
RC1_TRIM        1500
RC1_MAX         2000
RC2_MIN         1000
RC2_TRIM        1500
RC2_MAX         2000
RC3_MIN         1000
RC3_TRIM        1500
RC3_MAX         2000
RC4_MIN         1000
RC4_TRIM        1500
RC4_MAX         2000
"""


class SimulatorError(RuntimeError):
    """Raised when the simulator cannot be downloaded or started."""


class _BaseSimulator:
    """Common context-manager plumbing."""

    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    @property
    def connection_string(self):
        raise NotImplementedError

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False


class ArduPilotSimulator(_BaseSimulator):
    """Current ArduCopter, run straight from ArduPilot's published build."""

    def __init__(self, home=None, speedup=1, instance=0):
        self.home = home or DEFAULT_HOME
        self.speedup = speedup
        self.instance = instance
        self._process = None
        self._log_file = None
        self._log_path = None

    @property
    def port(self):
        # SITL puts SERIAL0 on 5760, stepping by 10 per instance.
        return 5760 + 10 * self.instance

    @property
    def connection_string(self):
        return f"tcp:127.0.0.1:{self.port}"

    # -- binary management -------------------------------------------------

    def _ensure_binary(self):
        """Download the SITL build on first use; reuse the cache afterwards."""
        if platform.system() != "Windows":
            raise SimulatorError(
                "The prebuilt 'stable' SITL binaries published here are Windows "
                "(Cygwin) builds. On Linux/macOS, build ArduPilot's own SITL and "
                "run it with Tools/autotest/sim_vehicle.py, then point this "
                "project at it with --connect udp:127.0.0.1:14550. "
                "Alternatively use --firmware legacy."
            )

        CACHE_DIR.mkdir(parents=True, exist_ok=True)
        binary = CACHE_DIR / "ArduCopter.exe"

        # The runtime DLLs must sit next to the executable.
        wanted = [(BINARY_NAME, binary)] + [
            (dll, CACHE_DIR / dll) for dll in CYGWIN_RUNTIME
        ]
        missing = [(name, dest) for name, dest in wanted if not dest.exists()]

        if missing:
            log.info("Downloading ArduCopter SITL (%d files, ~19 MB) to %s",
                     len(missing), CACHE_DIR)
            for name, dest in missing:
                url = f"{FIRMWARE_BASE}/{name}"
                log.debug("  fetching %s", url)
                try:
                    # Download beside the target, then move, so an interrupted
                    # run never leaves a truncated binary in the cache.
                    tmp = dest.with_suffix(dest.suffix + ".part")
                    urllib.request.urlretrieve(url, tmp)
                    os.replace(tmp, dest)
                except OSError as exc:
                    raise SimulatorError(
                        f"Could not download {url}: {exc}"
                    ) from exc
            log.info("Download complete")

        return binary

    # -- lifecycle ---------------------------------------------------------

    def start(self):
        binary = self._ensure_binary()

        if _port_in_use(self.port):
            raise SimulatorError(
                f"Port {self.port} is already in use -- another simulator is "
                f"probably still running. Stop it, or pass a different "
                f"--instance."
            )

        # SITL writes eeprom.bin and logs into its working directory.
        workdir = CACHE_DIR / f"run{self.instance}"
        workdir.mkdir(parents=True, exist_ok=True)
        self._log_path = workdir / "sitl.log"

        defaults = _ensure_copter_parm()

        home = "{},{},{},{}".format(*self.home)
        args = [
            str(binary),
            "--model", "quad",
            "--home", home,
            "--speedup", str(self.speedup),
            "--defaults", str(defaults),
            f"-I{self.instance}",
        ]

        log.info("Launching ArduCopter SITL (home=%s, speedup=%sx)", home, self.speedup)
        # Keep the simulator's own output; it is the only place startup errors
        # (bad arguments, missing DLLs, port clashes) are reported.
        self._log_file = open(self._log_path, "w", encoding="utf-8", errors="replace")
        self._process = subprocess.Popen(
            args,
            cwd=str(workdir),
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
        )

        self._await_ready()
        log.info("SITL ready on %s", self.connection_string)
        return self

    def _await_ready(self, timeout=60):
        """Wait for SITL to announce its serial port.

        Deliberately does *not* test the port by connecting to it: SITL's
        SERIAL0 accepts a single TCP client, so a liveness probe would consume
        the very connection DroneKit is about to make. Watching the log is both
        non-invasive and more informative when startup fails.
        """
        marker = f"SERIAL0 on TCP port {self.port}"
        deadline = time.time() + timeout

        while time.time() < deadline:
            if self._process.poll() is not None:
                raise SimulatorError(
                    f"SITL exited with code {self._process.returncode}. "
                    f"Its log is at {self._log_path}:\n{self._tail_log()}"
                )
            if marker in self._read_log():
                return
            time.sleep(0.5)

        self.stop()
        raise SimulatorError(
            f"SITL did not report '{marker}' within {timeout}s. "
            f"Its log is at {self._log_path}:\n{self._tail_log()}"
        )

    def _read_log(self):
        try:
            return self._log_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""

    def _tail_log(self, lines=15):
        return "\n".join(self._read_log().splitlines()[-lines:])

    def stop(self):
        if self._process is not None:
            log.info("Stopping SITL")
            self._process.terminate()
            try:
                self._process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                log.debug("SITL ignored terminate; killing")
                self._process.kill()
            self._process = None

        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None


class WslSimulator(_BaseSimulator):
    """Official ArduPilot SITL, run inside WSL.

    This is the reference way to run SITL: ArduPilot's own Linux build, driven
    by ArduPilot's own ``Tools/autotest/default_params/copter.parm``. Nothing
    here reimplements or second-guesses the upstream configuration.

    The simulator listens on TCP inside WSL and Windows reaches it over
    localhost, so the mission code connects exactly as it would to any other
    vehicle.

    A **source build is strongly preferred** and is used automatically when
    ``~/ardupilot/build/sitl/bin/arducopter`` exists. ArduPilot's *published*
    Linux SITL binary is linked against an older glibc and does not boot on
    recent distributions -- on Ubuntu 26.04 (glibc 2.43) its scheduler threads
    never start, and it sits printing "Waiting for internal clock bits to be
    set" and rebooting instead of producing heartbeats. That failure is not
    caused by the parameters or the command line; a bare ``--model quad``
    reproduces it.

    To build (about ten minutes on a modern machine)::

        cd ~/ardupilot
        Tools/environment_install/install-prereqs-ubuntu.sh -y   # needs sudo
        ./waf configure --board sitl && ./waf copter
    """

    def __init__(self, home=None, speedup=1, instance=0, distro=WSL_DISTRO):
        self.home = home or DEFAULT_HOME
        self.speedup = speedup
        self.instance = instance
        self.distro = distro
        self._process = None
        self._log_file = None
        self._log_path = None

    @property
    def port(self):
        return 5760 + 10 * self.instance

    @property
    def connection_string(self):
        return f"tcp:127.0.0.1:{self.port}"

    # -- WSL plumbing ------------------------------------------------------

    def _wsl(self, script, timeout=600, check=True):
        """Run a bash snippet inside the distro and return its stdout.

        The script is base64-encoded rather than passed as text. ``wsl.exe``
        rebuilds a command line when crossing into Linux, and a multi-line
        argument does not survive that intact -- variable expansions silently
        come back empty. Encoding it makes the payload a single opaque token.
        """
        payload = base64.b64encode(script.encode("utf-8")).decode("ascii")
        result = subprocess.run(
            ["wsl", "-d", self.distro, "--", "bash", "-lc",
             f"echo {payload} | base64 -d | bash"],
            capture_output=True, text=True, timeout=timeout,
        )
        if check and result.returncode != 0:
            raise SimulatorError(
                f"WSL command failed ({result.returncode}): "
                f"{(result.stderr or result.stdout).strip()[:400]}"
            )
        return result.stdout

    def _ensure_assets(self):
        """Make sure the binary and ArduPilot's parameter file are present.

        Both are fetched inside WSL on first use. The parameter file is taken
        from a checked-out ArduPilot tree when there is one, and downloaded
        from ArduPilot's repository otherwise -- either way it is upstream's
        file, not a local approximation of it.
        """
        script = f"""
set -e
DEST="{WSL_HOME_DIR}"
mkdir -p "$DEST"
if [ -x "{WSL_SOURCE_BINARY}" ]; then
  BIN="{WSL_SOURCE_BINARY}"
  echo "BIN_SOURCE=source build"
else
  BIN="$DEST/arducopter"
  echo "BIN_SOURCE=published binary"
  if [ ! -x "$BIN" ]; then
    echo "FETCHING_BINARY"
    if command -v curl >/dev/null; then curl -fsSL "{WSL_BINARY_URL}" -o "$BIN"
    elif command -v wget >/dev/null; then wget -q "{WSL_BINARY_URL}" -O "$BIN"
    else python3 -c "import urllib.request,sys; urllib.request.urlretrieve(sys.argv[1], sys.argv[2])" "{WSL_BINARY_URL}" "$BIN"
    fi
    chmod +x "$BIN"
  fi
fi
if [ -f "{WSL_REPO_PARM}" ]; then
  echo "PARM={WSL_REPO_PARM}"
  echo "PARM_SOURCE=ardupilot repository"
else
  PARM="$DEST/copter.parm"
  if [ ! -f "$PARM" ]; then
    echo "FETCHING_PARAMS"
    if command -v curl >/dev/null; then curl -fsSL "{COPTER_PARM_URL}" -o "$PARM"
    elif command -v wget >/dev/null; then wget -q "{COPTER_PARM_URL}" -O "$PARM"
    else python3 -c "import urllib.request,sys; urllib.request.urlretrieve(sys.argv[1], sys.argv[2])" "{COPTER_PARM_URL}" "$PARM"
    fi
  fi
  echo "PARM=$PARM"
  echo "PARM_SOURCE=ArduPilot master (downloaded)"
fi
echo "BIN=$BIN"
"""
        output = self._wsl(script)
        if "FETCHING_BINARY" in output:
            log.info("Downloaded the official ArduPilot SITL binary into WSL")
        if "FETCHING_PARAMS" in output:
            log.info("Downloaded ArduPilot's copter.parm into WSL")

        values = dict(
            line.split("=", 1) for line in output.splitlines() if "=" in line
        )
        log.info("Using ArduPilot default parameters from: %s",
                 values.get("PARM_SOURCE", "unknown"))

        binary_source = values.get("BIN_SOURCE", "unknown")
        log.info("SITL binary: %s", binary_source)
        if binary_source == "published binary":
            log.warning(
                "Using ArduPilot's published Linux SITL binary. It does not "
                "boot on recent glibc (it stalls at 'Waiting for internal "
                "clock bits'). If this run fails, build from source: "
                "cd ~/ardupilot && Tools/environment_install/"
                "install-prereqs-ubuntu.sh -y && ./waf configure --board sitl "
                "&& ./waf copter"
            )
        return values["BIN"], values["PARM"]

    # -- lifecycle ---------------------------------------------------------

    def start(self):
        if not _wsl_available(self.distro):
            raise SimulatorError(
                f"WSL distro {self.distro!r} is not available. Install it with "
                f"'wsl --install -d Ubuntu', or use --firmware windows."
            )

        binary, parm = self._ensure_assets()
        self._kill_stale()

        run_dir = f"{WSL_HOME_DIR}/run{self.instance}"
        home = "{},{},{},{}".format(*self.home)
        command = (
            f'mkdir -p "{run_dir}" && cd "{run_dir}" && '
            f'exec "{binary}" --model quad --home {home} '
            f'--speedup {self.speedup} --defaults "{parm}" -I{self.instance}'
        )

        CACHE_DIR.mkdir(parents=True, exist_ok=True)
        self._log_path = CACHE_DIR / f"wsl-sitl-{self.instance}.log"
        self._log_file = open(self._log_path, "w", encoding="utf-8", errors="replace")

        log.info("Launching ArduPilot SITL in WSL (home=%s, speedup=%sx)",
                 home, self.speedup)
        self._process = subprocess.Popen(
            ["wsl", "-d", self.distro, "--", "bash", "-lc", command],
            stdout=self._log_file, stderr=subprocess.STDOUT,
        )

        self._await_ready()
        log.info("SITL ready on %s", self.connection_string)
        return self

    def _await_ready(self, timeout=90):
        marker = f"SERIAL0 on TCP port {self.port}"
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._process.poll() is not None:
                raise SimulatorError(
                    f"SITL exited with code {self._process.returncode}:\n"
                    f"{self._tail_log()}"
                )
            if marker in self._read_log():
                return
            time.sleep(0.5)
        self.stop()
        raise SimulatorError(
            f"SITL did not report '{marker}' within {timeout}s:\n{self._tail_log()}"
        )

    def _read_log(self):
        try:
            return self._log_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""

    def _tail_log(self, lines=15):
        return "\n".join(self._read_log().splitlines()[-lines:])

    def _kill_stale(self):
        # Killing wsl.exe does not reap the Linux process it started, so the
        # simulator is stopped by name inside the distro.
        self._wsl("pkill -f 'ardupilot_sitl/arducopter' || true", timeout=60,
                  check=False)
        time.sleep(1)

    def stop(self):
        if self._process is not None:
            log.info("Stopping SITL")
            self._kill_stale()
            self._process.terminate()
            try:
                self._process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None


class LegacySimulator(_BaseSimulator):
    """ArduCopter 3.3 via the ``dronekit-sitl`` package."""

    def __init__(self, home=None, speedup=1, instance=0):
        self.home = home or DEFAULT_HOME
        self.speedup = speedup
        self.instance = instance
        self._sitl = None

    @property
    def connection_string(self):
        if self._sitl is None:
            raise SimulatorError("Simulator is not running; call start() first")
        return self._sitl.connection_string()

    def start(self):
        import dronekit_sitl

        home = "{},{},{},{}".format(*self.home)
        args = [
            "--model", "quad",
            "--home", home,
            "--speedup", str(self.speedup),
            "-I", str(self.instance),
        ]

        log.info("Launching ArduCopter 3.3 SITL (home=%s, speedup=%sx)",
                 home, self.speedup)
        binary = dronekit_sitl.SITL()
        binary.download("copter", "3.3", verbose=False)
        binary.launch(args, await_ready=True, restart=True)
        self._sitl = binary
        log.info("SITL ready on %s", self.connection_string)
        return self

    def stop(self):
        if self._sitl is None:
            return
        log.info("Stopping SITL")
        try:
            self._sitl.stop()
        except Exception as exc:
            # dronekit-sitl calls psutil.Process(pid) unconditionally, which
            # raises NoSuchProcess if the simulator already exited on its own
            # (common on Windows once the MAVLink client disconnects). Tearing
            # down a simulator must never fail an otherwise successful mission.
            log.debug("SITL was already gone: %s", exc)
        finally:
            self._sitl = None


BACKENDS = {
    "wsl": WslSimulator,
    "windows": ArduPilotSimulator,
    "legacy": LegacySimulator,
}


def _wsl_available(distro=WSL_DISTRO):
    """True if the named WSL distro exists and can run a command."""
    try:
        result = subprocess.run(
            ["wsl", "-d", distro, "--", "true"],
            capture_output=True, text=True, timeout=60,
        )
        return result.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def make_simulator(firmware="windows", **kwargs):
    """Build a simulator for the named firmware ('stable' or 'legacy')."""
    try:
        backend = BACKENDS[firmware]
    except KeyError:
        raise SimulatorError(
            f"Unknown firmware {firmware!r}; choose one of "
            f"{', '.join(sorted(BACKENDS))}"
        ) from None
    return backend(**kwargs)


def _ensure_copter_parm():
    """ArduPilot's own SITL parameter file, cached locally.

    Fetched from the ArduPilot repository so the vehicle is configured by
    exactly the file ``sim_vehicle.py`` would use. :data:`DEFAULT_PARAMS` is
    only a fallback for running with no network.
    """
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    dest = CACHE_DIR / "copter.parm"
    if dest.exists():
        return dest

    tmp = dest.with_suffix(".part")
    try:
        urllib.request.urlretrieve(COPTER_PARM_URL, tmp)
        os.replace(tmp, dest)
        log.info("Fetched ArduPilot's copter.parm -> %s", dest)
    except OSError as exc:
        log.warning("Could not fetch ArduPilot's copter.parm (%s); "
                    "falling back to the built-in minimal parameters", exc)
        dest.write_text(DEFAULT_PARAMS, encoding="utf-8")
    return dest


def _port_in_use(port, host="127.0.0.1"):
    """True if ``port`` is already bound.

    Tests by binding rather than connecting, so that probing never steals the
    single TCP client slot a waiting SITL instance is holding open.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        try:
            probe.bind((host, port))
            return False
        except OSError:
            return True


# Convenience alias for the default backend.
Simulator = WslSimulator
