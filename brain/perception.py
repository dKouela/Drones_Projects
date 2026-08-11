"""The perception thread.

Capture and inference run here, at whatever rate the camera and model manage.
The control loop never calls into this thread -- it reads
:attr:`Perception.latest`, which is always the most recent completed result.

That decoupling is the whole point. If inference takes 80 ms, the control loop
still emits setpoints every 50 ms; it simply acts on information that is one
frame old, which is far better than a vehicle left uncommanded while a model
runs.
"""

import logging
import threading
import time

from brain.latency import LatencyReport

log = logging.getLogger(__name__)


class Perception(threading.Thread):
    """Runs a :class:`~brain.frames.FrameSource` through a detector."""

    def __init__(self, source, detector, preview=False):
        super().__init__(daemon=True, name="perception")
        self.source = source
        self.detector = detector
        self.preview = preview

        self.latency = LatencyReport("capture->detected", "inference")
        self._latest = None
        self._latest_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._frames = 0
        self._detections = 0
        self._started_at = None

    # -- results ----------------------------------------------------------

    @property
    def latest(self):
        """The most recent :class:`~brain.detectors.Detection`, or ``None``.

        Returns ``None`` both before the first detection and after the target
        is lost; callers decide staleness from ``detection.captured_at``.
        """
        with self._latest_lock:
            return self._latest

    @property
    def stats(self):
        elapsed = max(time.perf_counter() - (self._started_at or 0), 1e-6)
        return {
            "frames": self._frames,
            "detections": self._detections,
            "fps": self._frames / elapsed,
            "hit_rate": self._detections / self._frames if self._frames else 0.0,
        }

    # -- thread -----------------------------------------------------------

    def run(self):
        self._started_at = time.perf_counter()
        log.info("Perception starting (source=%s detector=%s)",
                 self.source.name, self.detector.name)

        while not self._stop_event.is_set():
            frame, captured_at = self.source.read()
            if frame is None:
                # A dropped frame is normal on USB cameras; keep going.
                time.sleep(0.005)
                continue
            self._frames += 1

            started = time.perf_counter()
            try:
                detection = self.detector.detect(frame, captured_at)
            except Exception:
                log.exception("Detector raised; treating frame as empty")
                detection = None
            finished = time.perf_counter()

            self.latency["inference"].record((finished - started) * 1000.0)
            self.latency["capture->detected"].record((finished - captured_at) * 1000.0)

            if detection is not None:
                self._detections += 1
            with self._latest_lock:
                self._latest = detection

            if self.preview:
                self._show(frame, detection)

        log.info("Perception stopped after %d frames (%.1f fps, %.0f%% hit rate)",
                 self._frames, self.stats["fps"], self.stats["hit_rate"] * 100)

    def _show(self, frame, detection):
        import cv2

        from brain.detectors import draw

        cv2.imshow("brain", draw(frame, detection))
        cv2.waitKey(1)

    def stop(self):
        self._stop_event.set()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        self.join(timeout=3)
        self.source.close()
        self.detector.close()
        return False
