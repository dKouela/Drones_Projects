"""Autonomous drone mission runner (DroneKit + ArduCopter SITL).

    python main.py list
    python main.py run takeoff-land --alt 10
    python main.py run square --size 30 --alt 15
    python main.py run survey --width 100 --height 60 --spacing 20

By default a local ArduCopter SITL instance is started for the flight and shut
down afterwards. Pass --connect to talk to an already-running simulator or a
real vehicle instead.
"""

import argparse
import logging
import sys

import drone  # noqa: F401  -- must be imported before dronekit; installs shims
from brain import BrainError
from drone import control
from drone.connect import vehicle_session
from drone.control import FlightError
from drone.recorder import FlightRecorder
from drone.sitl import BACKENDS, DEFAULT_HOME, SimulatorError, make_simulator
from drone.telemetry import Reporter
from missions import REGISTRY

log = logging.getLogger("main")


def build_parser():
    # Options shared by every mission, attached via `parents=`.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--connect", dest="connection_string", default=None,
                        help="MAVLink endpoint (e.g. tcp:127.0.0.1:5760, "
                             "udp:127.0.0.1:14550, COM3). Omit to launch SITL.")
    common.add_argument("--firmware", choices=sorted(BACKENDS), default="windows",
                        help="which simulator to launch: 'windows' runs "
                             "ArduPilot's Cygwin SITL build natively, 'wsl' "
                             "runs a source-built SITL inside WSL, 'legacy' is "
                             "ArduCopter 3.3 (default: windows)")
    common.add_argument("--speedup", type=float, default=1.0,
                        help="SITL simulation rate multiplier (default: 1)")
    common.add_argument("--instance", type=int, default=0,
                        help="SITL instance number; use a different one to run "
                             "two simulators side by side (default: 0)")
    common.add_argument("--telemetry", type=float, default=0.0,
                        help="print a telemetry line every N seconds (0 = off)")
    common.add_argument("--record", metavar="PATH", default=None,
                        help="record the flight track to a JSON file, for "
                             "viewing afterwards with tools/flightview.py")
    common.add_argument("-v", "--verbose", action="store_true",
                        help="enable debug logging")

    parser = argparse.ArgumentParser(
        prog="main.py",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list", help="show the available missions")

    run = sub.add_parser("run", help="fly a mission")
    run_sub = run.add_subparsers(dest="mission", required=True)
    for name, module in sorted(REGISTRY.items()):
        mission_parser = run_sub.add_parser(
            name, parents=[common], help=module.DESCRIPTION,
        )
        module.add_arguments(mission_parser)

    return parser


def cmd_list():
    print("Available missions:\n")
    for name in sorted(REGISTRY):
        print(f"  {name:<14} {REGISTRY[name].DESCRIPTION}")
    print("\nRun one with:  python main.py run <mission> [options]")


def cmd_run(args):
    module = REGISTRY[args.mission]
    log.info("Mission: %s -- %s", args.mission, module.DESCRIPTION)

    simulator = None
    connection_string = args.connection_string

    if connection_string is None:
        simulator = make_simulator(
            args.firmware,
            home=DEFAULT_HOME,
            speedup=args.speedup,
            instance=args.instance,
        ).start()
        connection_string = simulator.connection_string
    else:
        log.warning("Using an external endpoint (%s). If this is a real "
                    "vehicle, make sure the area is clear.", connection_string)

    try:
        with vehicle_session(connection_string) as vehicle:
            reporter = Reporter(vehicle, args.telemetry) if args.telemetry else None
            if reporter:
                reporter.start()
            recorder = FlightRecorder(vehicle) if args.record else None
            if recorder:
                recorder.start()
            try:
                # Missions that plan relative to home must not read the
                # position before the EKF has produced one.
                control.wait_for_position(vehicle)
                module.run(vehicle, args)
                log.info("Mission '%s' completed successfully", args.mission)
            except KeyboardInterrupt:
                log.warning("Interrupted -- switching to RTL for a safe recovery")
                _emergency_rtl(vehicle)
                return 130
            finally:
                if reporter:
                    reporter.stop()
                if recorder:
                    recorder.stop()
                    recorder.join(timeout=3)
                    recorder.save(args.record, mission=args.mission,
                                  firmware=args.firmware)
    finally:
        if simulator:
            simulator.stop()

    return 0


def _emergency_rtl(vehicle):
    """Best-effort recovery after an interrupt; never raises."""
    try:
        from drone.control import return_to_launch
        return_to_launch(vehicle, timeout=120)
    except Exception as exc:
        log.error("RTL failed: %s -- take manual control!", exc)


def main(argv=None):
    args = build_parser().parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if getattr(args, "verbose", False) else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)-18s %(message)s",
        datefmt="%H:%M:%S",
    )
    # DroneKit and pymavlink are extremely chatty at DEBUG level.
    logging.getLogger("autopilot").setLevel(logging.WARNING)
    logging.getLogger("dronekit").setLevel(logging.WARNING)

    if args.command == "list":
        cmd_list()
        return 0

    try:
        return cmd_run(args)
    except FlightError as exc:
        log.error("Flight aborted: %s", exc)
        return 1
    except SimulatorError as exc:
        log.error("Simulator error: %s", exc)
        return 1
    except BrainError as exc:
        log.error("Perception error: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
