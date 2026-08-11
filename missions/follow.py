"""Take off, then visually track and follow a target.

This is the reactive counterpart to the other missions: nothing is planned in
advance, and the vehicle is flown by what the camera sees, at 20 Hz decisions
over a 50 Hz setpoint stream.
"""

import logging

from brain.detectors import create_detector
from brain.fastlink import FastLink
from brain.follow import FollowConfig, follow_target
from brain.frames import create_source
from brain.perception import Perception
from drone import control

DESCRIPTION = "Track a target with the camera and follow it (AI, high-rate control)"

log = logging.getLogger(__name__)


def add_arguments(parser):
    parser.add_argument("--alt", type=float, default=10.0,
                        help="altitude to hold while following (default: 10)")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="how long to follow, in seconds (default: 60)")
    parser.add_argument("--source", choices=("webcam", "synthetic"),
                        default="webcam",
                        help="where frames come from; 'synthetic' renders a "
                             "moving target so the loop runs with no camera "
                             "(default: webcam)")
    parser.add_argument("--detector", choices=("yunet", "color"), default="yunet",
                        help="'yunet' is a DNN face detector, 'color' finds the "
                             "largest blob of one hue (default: yunet)")
    parser.add_argument("--camera", type=int, default=0,
                        help="camera index for --source webcam (default: 0)")
    parser.add_argument("--target-size", type=float, default=0.35,
                        help="desired target height as a fraction of the frame; "
                             "larger means follow closer (default: 0.35)")
    parser.add_argument("--control-rate", type=float, default=20.0,
                        help="decisions per second (default: 20)")
    parser.add_argument("--setpoint-rate", type=float, default=50.0,
                        help="velocity setpoints streamed per second (default: 50)")
    parser.add_argument("--max-speed", type=float, default=4.0,
                        help="maximum forward speed in m/s (default: 4)")
    parser.add_argument("--max-radius", type=float, default=60.0,
                        help="never chase further than this from home, in "
                             "metres (default: 60)")
    parser.add_argument("--preview", action="store_true",
                        help="show the annotated camera view in a window")


def run(vehicle, args):
    # The camera runs in wall-clock time whatever the simulator does, so a
    # speedup silently changes the ratio of flight time to frames -- the
    # vehicle covers 5x the ground between decisions at --speedup 5. Useful for
    # the planned missions, misleading here.
    if getattr(args, "speedup", 1.0) != 1.0 and args.connection_string is None:
        log.warning("--speedup %g distorts a camera-driven loop: the vehicle "
                    "flies %gx further between frames than it would in "
                    "reality. Use --speedup 1 to judge the control tuning.",
                    args.speedup, args.speedup)

    source = create_source(args.source, index=args.camera)
    detector = create_detector(args.detector)

    config = FollowConfig(
        target_size=args.target_size,
        control_rate_hz=args.control_rate,
        max_forward=args.max_speed,
        max_radius_m=args.max_radius,
        min_altitude_m=min(3.0, args.alt / 2),
        max_altitude_m=args.alt * 2.5,
    )

    # Start perceiving before taking off: by the time the vehicle is at
    # altitude the detector is warm and the first frames are already behind us.
    with Perception(source, detector, preview=args.preview) as perception:
        control.arm_and_takeoff(vehicle, args.alt)

        link = FastLink(vehicle, rate_hz=args.setpoint_rate)
        link.request_fast_telemetry(20)
        with link:
            # Velocity setpoints are only accepted in GUIDED.
            control.set_mode(vehicle, "GUIDED")
            summary = follow_target(vehicle, perception, link,
                                    config=config, duration_s=args.duration)

    log.info("Follow finished: %s", summary["outcome"])
    log.info("  tracked %.1fs over %d control cycles; %d setpoints sent",
             summary["tracked_s"], summary["iterations"], summary["setpoints_sent"])
    log.info("  camera %.1f fps, target seen in %.0f%% of frames",
             summary["camera_fps"], summary["detection_rate"] * 100)
    for line in summary["latency_lines"]:
        log.info("  %s", line)

    control.return_to_launch(vehicle)
