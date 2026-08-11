"""Frame sources.

A source yields ``(frame, captured_at)`` where ``captured_at`` is a
``time.perf_counter()`` stamp taken as close to the grab as possible. Every
latency figure the brain reports is measured from that stamp, so it must not be
filled in later.
"""

import logging
import math
import time

import cv2
import numpy as np

from brain import BrainError

log = logging.getLogger(__name__)


class FrameSource:
    """Interface for anything that produces images."""

    #: Human-readable name, used in logs.
    name = "frames"

    def read(self):
        """Return ``(frame, captured_at)``, or ``(None, captured_at)`` on failure."""
        raise NotImplementedError

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


class WebcamSource(FrameSource):
    """Live capture from an attached camera.

    On a companion computer this is the class you replace -- a CSI camera via
    GStreamer, or an RTSP stream from a gimbal, exposes the same two methods.
    """

    name = "webcam"

    def __init__(self, index=0, width=640, height=480, fps=30):
        self.index = index
        # CAP_DSHOW avoids the ~2s DirectShow negotiation stall on Windows.
        backend = cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else cv2.CAP_ANY
        self._cap = cv2.VideoCapture(index, backend)
        if not self._cap.isOpened():
            raise BrainError(
                f"Could not open camera {index}. Check that a camera is "
                f"attached and not in use by another application, or pass "
                f"--source synthetic to run the same pipeline without one."
            )
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        # A large buffer means we would process stale frames; keep it minimal.
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        log.info("Camera %d opened at %dx%d",
                 index,
                 int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                 int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    def read(self):
        ok, frame = self._cap.read()
        captured_at = time.perf_counter()
        return (frame if ok else None), captured_at

    def close(self):
        self._cap.release()


class SyntheticSource(FrameSource):
    """A moving target rendered in software.

    Lets the whole perception-to-control loop be exercised, measured and
    regression-tested with no camera attached and no model download. The target
    circles the frame while its radius breathes, so both the yaw axis and the
    standoff-distance axis are driven.
    """

    name = "synthetic"

    def __init__(self, width=640, height=480, fps=30, period_s=12.0,
                 colour=(0, 0, 220)):
        self.width = width
        self.height = height
        self.fps = fps
        self.period_s = period_s
        self.colour = colour
        self._start = time.perf_counter()
        self._last_read = 0.0

    def read(self):
        # Pace ourselves so the synthetic source behaves like a real camera.
        interval = 1.0 / self.fps
        wait = self._last_read + interval - time.perf_counter()
        if wait > 0:
            time.sleep(wait)

        elapsed = time.perf_counter() - self._start
        phase = 2 * math.pi * elapsed / self.period_s

        frame = np.full((self.height, self.width, 3), 40, dtype=np.uint8)
        cx = int(self.width / 2 + 0.30 * self.width * math.sin(phase))
        cy = int(self.height / 2 + 0.12 * self.height * math.sin(phase * 0.7))
        radius = int(0.10 * self.height * (1.0 + 0.35 * math.sin(phase * 0.5)))
        cv2.circle(frame, (cx, cy), radius, self.colour, -1)

        captured_at = time.perf_counter()
        self._last_read = captured_at
        return frame, captured_at


def create_source(kind, **kwargs):
    """Build a frame source by name ('webcam' or 'synthetic')."""
    if kind == "webcam":
        return WebcamSource(**kwargs)
    if kind == "synthetic":
        kwargs.pop("index", None)
        return SyntheticSource(**kwargs)
    raise ValueError(f"Unknown frame source {kind!r}")
