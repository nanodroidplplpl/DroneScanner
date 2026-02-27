"""
Mouse-callback handler and host-side XYZ spatial calculation.

Spatial coordinates are computed entirely on the host using the camera's
calibration HFOV and the aligned depth map, matching the official Luxonis
``calc-spatial-on-host`` example:

    https://github.com/luxonis/oak-examples/tree/main/depth-measurement/calc-spatial-on-host

Two interaction gestures on a frozen snapshot are supported:

- **Point click** — left-click produces XYZ at a single pixel (tiny ROI).
- **ROI drag** — click-and-drag a rectangle for area-averaged XYZ.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------


@dataclass
class SpatialResult:
    """One XYZ measurement produced by :func:`compute_spatial_from_depth`.

    Attributes
    ----------
    x_mm, y_mm, z_mm
        3-D position relative to the camera centre (mm).
        X = right, Y = up, Z = forward.
    distance_mm
        Euclidean distance from the camera (mm).
    roi_px
        Pixel-space rectangle ``(x1, y1, x2, y2)`` used for drawing.
    """

    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    distance_mm: float = 0.0
    roi_px: tuple[int, int, int, int] | None = None


# ---------------------------------------------------------------------------
# Mouse interaction FSM
# ---------------------------------------------------------------------------


class InteractionState(Enum):
    """States of the click / drag finite-state machine."""

    IDLE = auto()
    DRAGGING = auto()
    DONE = auto()


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: Half-size (px) of the small ROI created around a single-point click.
POINT_ROI_HALF_SIZE: int = 5

#: If both dx and dy of a drag are below this threshold the gesture is
#: treated as a point click rather than a rectangle drag.
CLICK_THRESHOLD_PX: int = 5


# ---------------------------------------------------------------------------
# Host-side spatial calculation  (matches calc-spatial-on-host)
# ---------------------------------------------------------------------------


def compute_spatial_from_depth(
    depth_frame: np.ndarray,
    roi_px: tuple[int, int, int, int],
    hfov_deg: float,
    depth_thresh_low_mm: int = 200,
    depth_thresh_high_mm: int = 10_000,
) -> SpatialResult | None:
    """Compute X, Y, Z spatial coordinates from a depth-frame ROI.

    The maths mirrors the official Luxonis ``calc-spatial-on-host`` example::

        HFOV = deg2rad(calib.getFov(CAM_A))
        angle_x = atan(tan(HFOV / 2) * x_offset / (frame_w / 2))
        X = avg_depth * tan(angle_x)
        Y = -avg_depth * tan(angle_y)
        Z = avg_depth

    Parameters
    ----------
    depth_frame
        ``uint16`` depth map in millimetres, aligned to the RGB camera
        via ``StereoDepth.setDepthAlign(CAM_A)``.
    roi_px
        ``(x1, y1, x2, y2)`` pixel rectangle to measure.
    hfov_deg
        Horizontal field-of-view in **degrees** (from
        ``CalibrationHandler.getFov``).
    depth_thresh_low_mm, depth_thresh_high_mm
        Depth values outside ``[low, high]`` are excluded from the
        average.

    Returns
    -------
    SpatialResult | None
        ``None`` when the ROI contains no valid depth pixels.
    """
    x1, y1, x2, y2 = roi_px
    h, w = depth_frame.shape[:2]

    # Clamp ROI to frame bounds
    x1 = max(0, min(w, x1))
    y1 = max(0, min(h, y1))
    x2 = max(0, min(w, x2))
    y2 = max(0, min(h, y2))
    if x1 >= x2 or y1 >= y2:
        return None

    roi_depth = depth_frame[y1:y2, x1:x2]

    # Keep only pixels within the valid depth range
    in_range = (depth_thresh_low_mm <= roi_depth) & (roi_depth <= depth_thresh_high_mm)
    if not in_range.any():
        return None

    avg_depth: float = float(np.mean(roi_depth[in_range]))

    # ROI centroid in pixel space
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0

    # Offset from frame centre (used as normalised "sensor" position)
    mid_w = w / 2.0
    mid_h = h / 2.0

    # Angular projection (pin-hole camera model)
    hfov_rad = np.deg2rad(hfov_deg)
    angle_x = math.atan(math.tan(hfov_rad / 2.0) * (cx - mid_w) / mid_w)
    angle_y = math.atan(math.tan(hfov_rad / 2.0) * (cy - mid_h) / mid_h)

    x_mm = avg_depth * math.tan(angle_x)
    y_mm = -avg_depth * math.tan(angle_y)  # image-Y is downward; spatial-Y is upward
    z_mm = avg_depth
    distance_mm = math.sqrt(x_mm**2 + y_mm**2 + z_mm**2)

    return SpatialResult(
        x_mm=x_mm,
        y_mm=y_mm,
        z_mm=z_mm,
        distance_mm=distance_mm,
        roi_px=(x1, y1, x2, y2),
    )


def compute_spatial_matrix_from_depth(
    depth_frame: np.ndarray,
    hfov_deg: float,
    depth_thresh_low_mm: int = 200,
    depth_thresh_high_mm: int = 10_000,
    step: int = 1,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute full-frame X, Y, Z spatial matrices from a depth frame.

    Uses the same pin-hole projection as :func:`compute_spatial_from_depth`
    applied to every pixel (or every *step*-th pixel). Invalid depth values
    (0 or outside the threshold range) yield NaN so callers can mask them
    (e.g. for 3D heatmaps).

    Parameters
    ----------
    depth_frame
        ``uint16`` depth map in millimetres, aligned to the RGB camera.
    hfov_deg
        Horizontal field-of-view in degrees (from device calibration).
    depth_thresh_low_mm, depth_thresh_high_mm
        Depth values outside ``[low, high]`` are treated as invalid (NaN).
    step
        Sample every *step*-th row and column (1 = full resolution).

    Returns
    -------
    tuple of (X_mm, Y_mm, Z_mm)
        Three float arrays of shape ``(H_out, W_out)`` where
        ``H_out = (depth_frame.shape[0] + step - 1) // step`` and
        ``W_out = (depth_frame.shape[1] + step - 1) // step``.
        Invalid pixels are set to ``np.nan``.
    """
    h, w = depth_frame.shape[:2]
    if step < 1:
        step = 1

    # Subsample depth and build pixel coordinate grids
    depth_slice = depth_frame[::step, ::step]
    h_out, w_out = depth_slice.shape

    # Pixel coordinates: row i, col j in slice -> original (i*step, j*step)
    rows = np.arange(h_out, dtype=np.float64) * step
    cols = np.arange(w_out, dtype=np.float64) * step
    cy, cx = np.meshgrid(rows, cols, indexing="ij")

    mid_w = w / 2.0
    mid_h = h / 2.0
    hfov_rad = np.deg2rad(hfov_deg)
    tan_half_fov = math.tan(hfov_rad / 2.0)

    # Valid depth mask (same shape as depth_slice)
    depth_f64 = depth_slice.astype(np.float64)
    valid = (
        (depth_slice > 0)
        & (depth_slice >= depth_thresh_low_mm)
        & (depth_slice <= depth_thresh_high_mm)
    )

    # Angular projection (vectorized)
    angle_x = np.arctan(tan_half_fov * (cx - mid_w) / mid_w)
    angle_y = np.arctan(tan_half_fov * (cy - mid_h) / mid_h)

    x_mm = np.where(valid, depth_f64 * np.tan(angle_x), np.nan)
    y_mm = np.where(valid, -depth_f64 * np.tan(angle_y), np.nan)  # image-Y down
    z_mm = np.where(valid, depth_f64, np.nan)

    return (x_mm, y_mm, z_mm)


# ---------------------------------------------------------------------------
# Mouse handler
# ---------------------------------------------------------------------------


class MeasurementHandler:
    """OpenCV mouse-callback handler for point clicks and ROI drags.

    Usage::

        handler = MeasurementHandler(frame_w, frame_h)
        cv2.setMouseCallback(window_name, handler.mouse_callback)

    After each completed gesture, ``handler.pending_roi_px`` holds a
    pixel-space ``(x1, y1, x2, y2)`` tuple.  Call :meth:`consume_roi` to
    retrieve it and reset the handler for the next gesture.
    """

    def __init__(self, frame_w: int, frame_h: int) -> None:
        self.frame_w = frame_w
        self.frame_h = frame_h

        self._state = InteractionState.IDLE
        self._start_x = 0
        self._start_y = 0
        self._end_x = 0
        self._end_y = 0

        #: Pixel ROI ready for spatial calculation (``None`` = nothing pending).
        self.pending_roi_px: tuple[int, int, int, int] | None = None

        #: Live rubber-band rectangle while dragging (for visual feedback).
        self.drag_rect: tuple[int, int, int, int] | None = None

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------

    def mouse_callback(
        self,
        event: int,
        x: int,
        y: int,
        flags: int,
        param: object,
    ) -> None:
        """OpenCV mouse callback — pass directly to ``cv2.setMouseCallback``."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self._begin_drag(x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self._state == InteractionState.DRAGGING:
            self._update_drag(x, y)
        elif event == cv2.EVENT_LBUTTONUP and self._state == InteractionState.DRAGGING:
            self._finish_drag(x, y)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def consume_roi(self) -> tuple[int, int, int, int] | None:
        """Return the pending pixel ROI and reset the handler."""
        roi = self.pending_roi_px
        self.pending_roi_px = None
        self._state = InteractionState.IDLE
        return roi

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _begin_drag(self, x: int, y: int) -> None:
        self._state = InteractionState.DRAGGING
        self._start_x = self._end_x = x
        self._start_y = self._end_y = y
        self.drag_rect = None

    def _update_drag(self, x: int, y: int) -> None:
        self._end_x = x
        self._end_y = y
        self.drag_rect = (self._start_x, self._start_y, self._end_x, self._end_y)

    def _finish_drag(self, x: int, y: int) -> None:
        self._end_x = x
        self._end_y = y
        self._state = InteractionState.DONE
        self.drag_rect = None

        dx = abs(self._end_x - self._start_x)
        dy = abs(self._end_y - self._start_y)

        if dx < CLICK_THRESHOLD_PX and dy < CLICK_THRESHOLD_PX:
            # Treat as a point click → tiny ROI centred on the click position
            half = POINT_ROI_HALF_SIZE
            x1 = max(0, self._start_x - half)
            y1 = max(0, self._start_y - half)
            x2 = min(self.frame_w, self._start_x + half)
            y2 = min(self.frame_h, self._start_y + half)
        else:
            # Normalise so (x1, y1) is always the top-left corner
            x1 = min(self._start_x, self._end_x)
            y1 = min(self._start_y, self._end_y)
            x2 = max(self._start_x, self._end_x)
            y2 = max(self._start_y, self._end_y)

        self.pending_roi_px = (x1, y1, x2, y2)
