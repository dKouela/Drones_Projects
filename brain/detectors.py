"""Detectors.

Two backends ship here:

``yunet``  A 227 KB deep-learning face detector from OpenCV's model zoo, run
           through ``cv2.FaceDetectorYN``. OpenCV owns the pre/post-processing,
           so there is no hand-rolled anchor decoding to get subtly wrong.
``color``  An HSV blob finder. No model, no download, microseconds per frame --
           the reference detector for testing the control loop in isolation.

Swapping in something heavier (YOLO, NanoDet, a TensorRT engine on a Jetson,
a Hailo graph on a Pi) means implementing :meth:`Detector.detect` and nothing
else; the perception and control layers never learn what produced a box.
"""

import logging
import os
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from brain import BrainError

log = logging.getLogger(__name__)

MODEL_DIR = Path.home() / ".drone_brain" / "models"

YUNET_URL = (
    "https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/"
    "face_detection_yunet_2023mar.onnx"
)


@dataclass
class Detection:
    """One detected target, in pixel coordinates."""

    x: float
    y: float
    w: float
    h: float
    confidence: float
    label: str
    frame_width: int
    frame_height: int
    #: perf_counter stamp of the frame this came from -- the latency origin.
    captured_at: float

    @property
    def cx(self):
        return self.x + self.w / 2

    @property
    def cy(self):
        return self.y + self.h / 2

    @property
    def offset_x(self):
        """Horizontal error from frame centre, normalised to [-1, 1]."""
        return (self.cx - self.frame_width / 2) / (self.frame_width / 2)

    @property
    def offset_y(self):
        """Vertical error from frame centre, normalised to [-1, 1] (down positive)."""
        return (self.cy - self.frame_height / 2) / (self.frame_height / 2)

    @property
    def size_ratio(self):
        """Box height as a fraction of frame height -- the range proxy."""
        return self.h / self.frame_height


class Detector:
    """Interface for anything that finds a target in a frame."""

    name = "detector"

    def detect(self, frame, captured_at):
        """Return the best :class:`Detection`, or ``None``."""
        raise NotImplementedError

    def close(self):
        pass


class ColorBlobDetector(Detector):
    """Largest blob of a given hue. Deterministic, model-free, ~1 ms."""

    name = "color"

    def __init__(self, hue=0, hue_tolerance=12, min_saturation=120,
                 min_value=70, min_area_px=200):
        self.hue = hue
        self.hue_tolerance = hue_tolerance
        self.min_saturation = min_saturation
        self.min_value = min_value
        self.min_area_px = min_area_px

    def _mask(self, hsv):
        lo_s, lo_v = self.min_saturation, self.min_value
        # Hue is circular, so a band around red (0) wraps and needs two ranges.
        low = (self.hue - self.hue_tolerance) % 180
        high = (self.hue + self.hue_tolerance) % 180
        if low <= high:
            return cv2.inRange(hsv, (low, lo_s, lo_v), (high, 255, 255))
        return cv2.bitwise_or(
            cv2.inRange(hsv, (0, lo_s, lo_v), (high, 255, 255)),
            cv2.inRange(hsv, (low, lo_s, lo_v), (179, 255, 255)),
        )

    def detect(self, frame, captured_at):
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._mask(hsv)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)
        if area < self.min_area_px:
            return None

        x, y, w, h = cv2.boundingRect(best)
        # Fill ratio stands in for confidence: a solid blob scores near 1.
        confidence = float(min(1.0, area / max(w * h, 1)))
        return Detection(x, y, w, h, confidence, "blob",
                         width, height, captured_at)


class YuNetFaceDetector(Detector):
    """DNN face detection via OpenCV's bundled YuNet runner."""

    name = "yunet"

    def __init__(self, score_threshold=0.7, nms_threshold=0.3, model_path=None):
        if not hasattr(cv2, "FaceDetectorYN"):
            raise BrainError(
                "This OpenCV build has no FaceDetectorYN; use --detector color."
            )
        self.model_path = Path(model_path) if model_path else _ensure_model(
            YUNET_URL, "face_detection_yunet_2023mar.onnx"
        )
        self._score_threshold = score_threshold
        self._detector = cv2.FaceDetectorYN.create(
            str(self.model_path), "", (320, 320), score_threshold, nms_threshold
        )
        self._input_size = None
        log.info("YuNet loaded from %s", self.model_path)

    def detect(self, frame, captured_at):
        height, width = frame.shape[:2]
        if self._input_size != (width, height):
            # The network is resized to the stream, not the stream to the
            # network -- no letterboxing, no coordinate rescaling afterwards.
            self._detector.setInputSize((width, height))
            self._input_size = (width, height)

        _, faces = self._detector.detect(frame)
        if faces is None or len(faces) == 0:
            return None

        # Columns 0-3 are the box, the last column is the score.
        best = max(faces, key=lambda f: f[-1])
        x, y, w, h = (float(v) for v in best[:4])
        return Detection(x, y, w, h, float(best[-1]), "face",
                         width, height, captured_at)


def _ensure_model(url, filename):
    """Download a model on first use and cache it under ~/.drone_brain."""
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    dest = MODEL_DIR / filename
    if dest.exists():
        return dest

    log.info("Downloading detector model to %s", dest)
    tmp = dest.with_suffix(dest.suffix + ".part")
    try:
        urllib.request.urlretrieve(url, tmp)
        os.replace(tmp, dest)
    except OSError as exc:
        raise BrainError(f"Could not download {url}: {exc}") from exc
    log.info("Model ready (%.0f KB)", dest.stat().st_size / 1024)
    return dest


def create_detector(kind, **kwargs):
    """Build a detector by name ('yunet' or 'color')."""
    if kind == "yunet":
        return YuNetFaceDetector(**kwargs)
    if kind == "color":
        return ColorBlobDetector(**kwargs)
    raise ValueError(f"Unknown detector {kind!r}")


def draw(frame, detection):
    """Annotate a frame in place (used by --preview)."""
    if detection is None:
        cv2.putText(frame, "no target", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return frame
    x, y, w, h = (int(v) for v in (detection.x, detection.y, detection.w, detection.h))
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 220, 0), 2)
    cv2.putText(frame, f"{detection.label} {detection.confidence:.2f}",
                (x, max(y - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
    return frame
