# Development environment

The project's goal is a **VTOL QuadPlane** that flies A→B, identifies vehicles and
people with an onboard open-source AI brain, avoids obstacles, and reports back —
built and tested entirely in simulation before any airframe exists.

This document sets up the environment that all of that runs in.

## The model: one Linux, reached from two machines

You work across **macOS** (Apple Silicon) and **Windows**, and prefer Linux. Rather
than fight two native toolchains (Gazebo runs poorly on both), the project lives in
**one Ubuntu 24.04 Linux environment**, and each machine hosts its own copy:

| Machine | Linux host | Architecture |
|---|---|---|
| Windows | **WSL2** running Ubuntu 24.04 | x86_64 |
| Mac (Apple Silicon) | **VMware Fusion** VM, Ubuntu 24.04 | ARM64 |

**Git is the bridge.** The repo lives at `github.com/dKouela/Drones_Projects`. You
`git push` from one machine and `git pull` on the other; only committed code moves.
The two Linux installs are never synced directly — each is built once by the same
setup script and kept current through git. ArduPilot SITL and Gazebo both run on
x86_64 and ARM64, so the setup is identical on both; apt resolves the architecture.

VS Code connects natively to either (Remote-WSL on Windows, Remote-SSH to the VM on
Mac), so editing feels local no matter which laptop you're on.

## First-time setup

### Windows

```powershell
wsl --install -d Ubuntu-24.04     # skip if WSL2 + Ubuntu already installed
```

Then open the Ubuntu shell and continue at "Inside Ubuntu" below.

### Mac (Apple Silicon) — VMware Fusion

Fusion on Apple Silicon runs **ARM64** guests only, so use the ARM64 ISO.

1. Download the **Ubuntu 24.04 LTS ARM64** ISO (the "ARM64" / "arm64" server or
   desktop image — *not* the amd64/x86_64 one).
2. In Fusion: **New → Install from disc or image →** select the ISO. Fusion
   detects it as ARM64 Ubuntu.
3. Before finishing, click **Customize Settings** and give it headroom for the
   sim build: **≥4 CPU cores, ≥8 GB RAM, ≥40 GB disk** (ArduPilot + Gazebo are
   large). More cores makes the `./waf` build much faster.
4. Install **VMware Tools / open-vm-tools** after first boot for a shared
   clipboard, resizable display, and folder sharing:
   `sudo apt-get install -y open-vm-tools open-vm-tools-desktop`
5. Continue at "Inside Ubuntu" below.

### Inside Ubuntu (both machines — identical)

```bash
git clone https://github.com/dKouela/Drones_Projects.git
cd Drones_Projects
./scripts/setup_linux.sh
```

That script is idempotent — safe to re-run. It installs build tools + a Python
3.12 venv, builds **ArduPilot SITL with the ArduPlane (QuadPlane) vehicle**, and
builds **Gazebo Harmonic + the ardupilot_gazebo plugin** for camera-in-the-loop
simulation. Pass `--no-gazebo` to skip the large Gazebo step on a first pass.

When it finishes, add the two `GZ_SIM_*` exports it prints to your `~/.bashrc`.

## First flight (bare QuadPlane, no brain yet)

This proves the simulator and the VTOL vehicle work, before any perception is added.

```bash
# terminal 1 — ArduPlane SITL as a QuadPlane
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane --console --map
```

In the SITL console you can arm, take off vertically, transition to forward flight,
fly a waypoint mission, and land — the same MAVLink this repo's missions already
speak. Point a mission at it with `--connect udp:127.0.0.1:14550`.

Gazebo (the rendered world with cameras) is wired in the next step, once the bare
QuadPlane flies. That's where the brain's perception gets developed against a
simulated desert scene with a vehicle and a person to detect.

## Why this order

The build follows the same risk-reducing sequence the real aircraft will:

1. **Bare QuadPlane in SITL** — prove transition and navigation (you are here).
2. **Gazebo + cameras** — give the brain something to see.
3. **Brain as passive payload** — perception + reporting, not yet steering.
4. **Brain with limited control** — GUIDED nudges inside the safety envelope.
5. **Avoidance + full autonomy** — added last, on proven layers beneath it.

Steps 2–5 are all developed here in simulation and port to the real Pi 5 + VTOL
unchanged, because the flight controller, MAVLink, and the repo's
`FrameSource`/`Detector`/`FastLink` interfaces stay the same across sim and hardware.
