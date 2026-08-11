# Autonomous Drone Missions — DroneKit + ArduCopter SITL

A working autonomous flight stack you can run entirely on your laptop. Missions
are written in Python against [DroneKit](https://github.com/dronekit/dronekit-python)
and flown by a real [ArduCopter](https://ardupilot.org/copter/) autopilot running
in software-in-the-loop simulation — the same firmware and the same MAVLink
protocol a real vehicle uses.

Flies **ArduCopter 4.7.0**, verified end to end on Windows 11 / Python 3.12.

## Quick start

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt

.\.venv\Scripts\python.exe main.py list
.\.venv\Scripts\python.exe main.py run takeoff-land --alt 10
```

The first run downloads the ArduCopter SITL build (~19 MB) into
`~/.ardupilot_sitl`. Everything after that is offline. No simulator needs to be
started by hand — `main.py` launches one and shuts it down when the mission ends.

### Which firmware you are flying

`--firmware` picks the simulator backend:

- **`stable`** (default) — current ArduCopter 4.x, taken from ArduPilot's own
  firmware server. These are the Cygwin builds Mission Planner ships, so they
  run natively on Windows with no WSL, Docker or toolchain.
- **`legacy`** — ArduCopter 3.3 (2015) via the `dronekit-sitl` package. Kept as
  a fallback; it is what most DroneKit tutorials target, and it ignores several
  MAVLink commands current firmware accepts.

Both are tested. On Linux/macOS the `stable` binaries do not apply — build
ArduPilot's own SITL, run `Tools/autotest/sim_vehicle.py`, and point this
project at it with `--connect udp:127.0.0.1:14550`.

## Missions

| Mission | What it does | Technique |
|---|---|---|
| `takeoff-land` | Climb, hover, land in place | Smoke test |
| `square` | Fly a square, then return home | GUIDED — Python steers |
| `waypoints` | Circular route around home | AUTO — autopilot flies uploaded plan |
| `survey` | Lawnmower grid over an area | AUTO — mapping/photogrammetry pattern |
| `follow` | Track a target and chase it | Onboard AI + high-rate velocity control |

```powershell
.\.venv\Scripts\python.exe main.py run square    --size 30 --alt 15 --speed 8
.\.venv\Scripts\python.exe main.py run waypoints --radius 40 --points 6
.\.venv\Scripts\python.exe main.py run survey    --width 100 --height 60 --spacing 20
.\.venv\Scripts\python.exe main.py run follow    --duration 60 --preview
```

Useful flags on every mission:

- `--speedup 10` — run the simulation 10x faster than real time
- `--telemetry 2` — print altitude/speed/heading/battery every 2 seconds
- `--firmware legacy` — fly ArduCopter 3.3 instead of current stable
- `--instance 1` — use a second simulator (port 5770) so two can run at once
- `--connect <endpoint>` — use an existing vehicle instead of launching SITL
- `-v` — debug logging

## The two ways to fly autonomously

This is the core concept the project is built around, and the reason `square`
and `waypoints` both exist.

**GUIDED** (`square`) — your Python script is in the loop. It sends one position
target at a time and decides the next one after the vehicle arrives. This is
what you want for reactive behaviour: following an object, avoiding something a
camera saw, adapting a search pattern. The cost is that the vehicle depends on a
live link — if your script dies, the vehicle just hovers.

**AUTO** (`waypoints`, `survey`) — the whole route is uploaded to the autopilot
up front, then you switch to AUTO and the flight controller executes it alone.
This survives a lost link or a crashed companion computer, which is why real
survey and delivery sorties are flown this way. The cost is that the plan is
fixed once it is running.

Both missions end with a return-to-launch, which is a third thing again: a
failsafe behaviour built into the firmware.

## The onboard brain

`brain/` is the reactive half of the project: perception and closed-loop
control, flown by what the camera sees rather than by a plan.

```powershell
# Follow a face with a webcam, watching the annotated view
.\.venv\Scripts\python.exe main.py run follow --detector yunet --preview

# No camera? Same pipeline against a rendered moving target
.\.venv\Scripts\python.exe main.py run follow --source synthetic --detector color
```

### Why not just use `simple_goto`

DroneKit's `simple_goto` sends a `MISSION_ITEM` — a "fly there eventually"
instruction that current firmware warns about on every call. It is the wrong
shape for closing a visual loop.

`brain/fastlink.py` instead streams `SET_POSITION_TARGET_LOCAL_NED` velocity
setpoints in the vehicle's body frame at 50 Hz. Two properties drive that design:

- ArduCopter treats guided velocity setpoints as perishable and stops the
  vehicle when they go stale (`GUID_TIMEOUT`, 3 s). Setpoints must be repeated
  whether or not the decision changed.
- The rate the vehicle is *commanded* at should not be tied to how fast
  decisions are *made*. A streaming thread re-sends the latest setpoint at a
  fixed rate; the controller replaces it whenever it has something new.

The same mechanism is the safety interlock: if nothing refreshes the setpoint
within 0.5 s, the streamer commands zero velocity rather than flying on a stale
intention. A crashed perception thread makes the drone hover, not run away.

### Measured latency

From the frame-grab instant to the setpoint reaching the vehicle, over a 25 s
flight (colour detector, 640×480, CPU only):

| Stage | mean | p95 | max |
|---|---|---|---|
| capture → detected | 1.5 ms | 2.0 ms | 3.5 ms |
| capture → command sent | 18.6 ms | 34.1 ms | 35.7 ms |
| setpoint interval (50 Hz target) | 20.0 ms | 20.6 ms | 21.0 ms |

The YuNet DNN costs ~10 ms per frame instead of 1.5 ms, which moves the mean to
roughly 27 ms. The gap between the 18.6 ms mean and the 34 ms tail is the 20 Hz
control beat, not the model: a detection waits up to one control period before
being acted on. Raise `--control-rate` to trade CPU for a shorter tail.

Percentiles are reported rather than averages because the tail is what makes a
drone hit something.

### How the following works

Three independent proportional loops, each driven by one property of the box:

| Box property | Drives | Effect |
|---|---|---|
| horizontal offset | yaw rate | turn to face the target |
| height vs `--target-size` | forward speed | hold a standoff distance |
| vertical offset | climb rate | keep it level in frame |

Proportional only, deliberately: a derivative term on a jittery detector turns
box noise into throttle noise, and an integral term winds up during the seconds
the target is not visible. Both become worth revisiting behind a tracker that
produces smooth, gap-free estimates.

Safety envelope, all enforced every cycle: minimum and maximum altitude, a
maximum chase radius from home (`--max-radius`), hover when the detection is
older than 0.6 s, and abort to RTL after 6 s without a target.

### Swapping the pieces

Each hardware-specific edge is behind an interface, which is what makes this
portable to a real companion computer:

| Interface | Ships with | Replace with |
|---|---|---|
| `FrameSource` | webcam, synthetic | CSI camera via GStreamer, RTSP from a gimbal |
| `Detector` | YuNet DNN, colour blob | YOLO/NanoDet ONNX, a TensorRT engine, a Hailo graph |
| `FastLink` | MAVLink over TCP/UDP | the same, over a serial link to a Pixhawk |

Only `Detector.detect()` needs implementing to change models — nothing
downstream learns what produced a box. On a Jetson the natural step is
exporting to ONNX and running it through TensorRT; `brain/detectors.py` already
loads ONNX, so the model file changes and the interface does not.

### Moving to real hardware

The link between the brain and the flight controller is what "high speed
transmission" has to mean in practice, and it is *not* the telemetry radio:

- Wire the companion computer to a Pixhawk **serial/TELEM port at 921600 baud**
  and set `SERIALn_PROTOCOL=2`, or use Ethernet on boards that have it. A 57600
  baud SiK radio cannot carry a 50 Hz setpoint stream — that link is for the
  ground station, not for control.
- `FastLink.request_fast_telemetry()` raises the rate of the messages the
  controller actually reads via `MAV_CMD_SET_MESSAGE_INTERVAL`, rather than
  relying on default stream rates.
- Keep perception and control on the companion computer. Anything that has to
  reach the ground and come back is not a fast reaction loop.

### What is verified, and what is not

Verified in SITL: the full loop end to end (perception → decision → setpoint →
vehicle motion), every axis of body-frame velocity control against measured
displacement and heading change, the stale-setpoint watchdog, the target-lost
hover and abort path, and the latency figures above. YuNet was checked against a
real photograph and detects a face at 0.91 confidence in ~9 ms.

**Not verified: live webcam capture.** The machine this was built on has no
camera attached, so `WebcamSource` is written but untested — the loop was
exercised through `--source synthetic` instead. If `--source webcam` misbehaves,
that class is the first place to look.

Also note that `--speedup` distorts this mission specifically. The camera runs
in wall-clock time no matter what the simulator does, so at `--speedup 5` the
vehicle covers five times the ground between frames. The other missions are
unaffected; `follow` warns when you do it.

## Layout

```
main.py               CLI entry point and mission runner
drone/
  compat.py           makes DroneKit import on Python 3.10+
  sitl.py             both simulator backends (stable / legacy)
  connect.py          MAVLink connection handling
  control.py          arm, takeoff, goto, mission upload, land, RTL
  geo.py              metre-offset and distance maths
  telemetry.py        telemetry snapshots + background reporter
brain/
  frames.py           where images come from (webcam, synthetic)
  detectors.py        what is in them (YuNet DNN, colour blob)
  perception.py       capture + inference thread
  fastlink.py         50 Hz MAVLink velocity/yaw-rate control
  follow.py           detection -> velocity command, safety envelope
  latency.py          percentile timing
missions/             one module per mission
```

### Writing a new mission

Drop a module in `missions/` exposing three things, then register it in
`missions/__init__.py`:

```python
DESCRIPTION = "What it does, one line"

def add_arguments(parser):
    parser.add_argument("--alt", type=float, default=10.0)

def run(vehicle, args):
    control.arm_and_takeoff(vehicle, args.alt)
    control.goto_offset(vehicle, north_m=50, east_m=0)
    control.return_to_launch(vehicle)
```

`vehicle` is a connected DroneKit `Vehicle`, so the full API
(`vehicle.battery`, `vehicle.attitude`, `vehicle.parameters[...]`, message
listeners) is available alongside the helpers in `drone/control.py`.

## Watching a flight on a map

The mission log prints coordinates, but a moving map is much easier to read.
MAVProxy is installed with the requirements — start SITL yourself, let MAVProxy
rebroadcast the link, and point the mission at it:

```powershell
# terminal 1 — simulator (the binary main.py already downloaded)
& "$env:USERPROFILE\.ardupilot_sitl\copter\ArduCopter.exe" --model quad `
    --home -35.363261,149.165230,584,353 `
    --defaults "$env:USERPROFILE\.ardupilot_sitl\copter\run0\defaults.parm"

# terminal 2 — ground station, fanning the link out to two UDP clients
.\.venv\Scripts\python.exe .\.venv\Scripts\mavproxy.py --master tcp:127.0.0.1:5760 `
    --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 --map --console

# terminal 3 — fly against the running simulator
.\.venv\Scripts\python.exe main.py run survey --connect udp:127.0.0.1:14551
```

Two `--out` endpoints are needed because two clients cannot share one MAVLink
stream: MAVProxy's own map takes the link, `14550` is free for
[Mission Planner](https://ardupilot.org/planner/) or QGroundControl, and `14551`
is the one your mission connects to. MAVProxy installs as `mavproxy.py` rather
than an `.exe`, hence invoking it through `python.exe`.

## Known quirks of this stack

DroneKit's last release was 2020, so pairing it with 2026 firmware needs care.
Each of these caused a real failure while building this, and each is handled in
the code — they are documented so the workarounds are not mistaken for noise.

**Affecting everything**

1. **DroneKit does not import on Python 3.10+.** It subclasses
   `collections.MutableMapping`, an alias removed in 3.10, and imports
   `past.builtins`. `drone/compat.py` restores the aliases and `future`
   supplies `past`. Import `drone` before `dronekit` anywhere you use it.

2. **`wait_ready=True` does not wait for position.** It covers parameters,
   mode, armed state, GPS and attitude — but not location, so lat/lon read
   `0.0` for the first few seconds after connecting. A mission that plans a
   route relative to home will lay it out at 0°N 0°E and fly out to sea.
   `control.wait_for_position()` blocks until the EKF produces a real fix, and
   `main.py` calls it before any mission starts.

3. **DroneKit's "dummy home waypoint" advice is wrong on current firmware.**
   Its examples prepend a placeholder command on the theory that sequence 0 is
   consumed as home. ArduCopter 4.x keeps home separately and executes the
   placeholder as a real waypoint to 0°N 0°E. `control.upload_mission()` sends
   no placeholder, then reads the mission back and takes the RTL command's
   actual sequence number from the autopilot instead of assuming the numbering.

**Affecting `--firmware stable` (ArduCopter 4.x)**

4. **A stock parameter set will not arm.** No frame is configured
   (`Motors: Check frame class and type`) and the simulated IMUs read as
   uncalibrated (`3D Accel calibration needed`). `drone/sitl.py` writes a
   defaults file mirroring ArduPilot's `Tools/autotest/default_params/copter.parm`.
   Note that `INS_ACC3*` must stay at zero: SITL has two accelerometers, and
   marking a third as calibrated makes ArduPilot reject the calibration outright.

5. **SITL's serial port accepts exactly one TCP client.** Probing the port to
   check whether the simulator is up consumes the connection DroneKit needs, so
   startup is detected by watching SITL's log for `SERIAL0 on TCP port`, and the
   "port already in use" check binds rather than connects.

**Affecting `--firmware legacy` (ArduCopter 3.3)**

6. **Mode changes silently do nothing.** DroneKit delegates to pymavlink, which
   now sends `MAV_CMD_DO_SET_MODE` as a COMMAND_LONG. ArduCopter 3.3 predates
   that and ignores it, so the vehicle stays in STABILIZE forever.
   `control.set_mode()` sends the legacy `SET_MODE` message instead, which both
   old and current ArduPilot understand.

7. **`dronekit-sitl` crashes on shutdown** if the simulator already exited — it
   calls `psutil.Process(pid)` unconditionally. `drone/sitl.py` swallows it.

One warning is expected and harmless: current firmware logs
`got MISSION_ITEM; GCS should send MISSION_ITEM_INT` during upload. DroneKit
only speaks the older float-based mission protocol, which costs roughly a metre
of waypoint precision — irrelevant in simulation, worth knowing outdoors.

## Flying real hardware

`--connect` accepts a serial port (`COM3`, `/dev/ttyUSB0`) or a network
endpoint, so these missions will fly a real vehicle unchanged. Before you try:

- Test the exact mission in SITL first, at the exact parameters.
- Keep a transmitter in hand with a mode switch — flipping to STABILIZE or LOITER
  overrides anything this code is doing.
- Check `--alt` against local airspace rules and your battery's real endurance.
- `arm_and_takeoff` deliberately waits for `vehicle.is_armable` (EKF converged,
  GPS lock) rather than forcing arming, and every blocking helper times out
  instead of hanging. Do not remove those guards to "make it work" outdoors.
