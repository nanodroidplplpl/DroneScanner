"""Stereo camera source returning per-pixel distance matrix (mm)."""

from __future__ import annotations

import time
from typing import Callable

import numpy as np

try:
    # Works when imported from project root, e.g. `from data_source.stereo_camera ...`
    from data_source.measurement import compute_spatial_matrix_from_depth
    from data_source.pipeline import CameraPipeline, PipelineConfig
except ModuleNotFoundError:
    # Works when executed directly, e.g. `python data_source/stereo_camera.py`
    from measurement import compute_spatial_matrix_from_depth
    from pipeline import CameraPipeline, PipelineConfig


class StereoCameraSource:
    """Thin camera wrapper with no display, returning numpy distance matrices.

    Distances are returned in millimeters as a 2D array (H, W).
    Invalid pixels are represented as ``np.nan``.
    """

    def __init__(
        self,
        config: PipelineConfig | None = None,
        frame_timeout_s: float = 2.0,
    ) -> None:
        self.config = config or PipelineConfig()
        self.frame_timeout_s = frame_timeout_s
        self._pipe = CameraPipeline(self.config)
        self._pipe.start()

    def close(self) -> None:
        """Release device resources."""
        self._pipe.stop()

    def __enter__(self) -> StereoCameraSource:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def get_stereo_camera_data(self) -> np.ndarray:
        """Return per-pixel camera distance matrix in millimeters.

        Returns:
            np.ndarray: 2D float matrix where each element is the Euclidean
                distance from camera center for a pixel.
                Invalid pixels are ``np.nan``.

        Raises:
            TimeoutError: If no new depth frame arrives before timeout.
            RuntimeError: If pipeline is not running.
        """
        if not self._pipe.is_running() or self._pipe.q_depth is None:
            raise RuntimeError("Stereo camera pipeline is not running.")

        deadline = time.monotonic() + self.frame_timeout_s
        depth_frame: np.ndarray | None = None

        while time.monotonic() < deadline:
            depth_msg = self._pipe.q_depth.tryGet()
            if depth_msg is not None:
                depth_frame = depth_msg.getFrame()
                break
            time.sleep(0.01)

        if depth_frame is None:
            raise TimeoutError(
                f"Did not receive depth frame within {self.frame_timeout_s:.1f}s."
            )

        x_mm, y_mm, z_mm = compute_spatial_matrix_from_depth(
            depth_frame=depth_frame,
            hfov_deg=self._pipe.hfov_deg,
            depth_thresh_low_mm=self.config.depth_lower_threshold_mm,
            depth_thresh_high_mm=self.config.depth_upper_threshold_mm,
            step=1,
        )
        distance_mm = np.sqrt(x_mm**2 + y_mm**2 + z_mm**2, dtype=np.float64)
        return distance_mm


_DEFAULT_CAMERA_SOURCE: StereoCameraSource | None = None


def get_stereo_camera_data() -> np.ndarray:
    """Compatibility helper used by existing scripts.

    Lazily creates one shared camera source and returns one distance matrix.
    """
    global _DEFAULT_CAMERA_SOURCE
    if _DEFAULT_CAMERA_SOURCE is None:
        _DEFAULT_CAMERA_SOURCE = StereoCameraSource()
    return _DEFAULT_CAMERA_SOURCE.get_stereo_camera_data()


def _block_reduce_distance_matrix(
    distance_matrix: np.ndarray,
    block_h: int,
    block_w: int,
    reducer: Callable[[np.ndarray], float],
) -> np.ndarray:
    """Downsample a 2D distance matrix by reducing non-overlapping blocks.

    The output size is:
        (ceil(H / block_h), ceil(W / block_w))

    ``np.nan`` values are ignored inside each block.
    If a whole block has no valid values, output pixel is ``np.nan``.
    """
    if distance_matrix.ndim != 2:
        raise ValueError(
            f"distance_matrix must be 2D, got shape={distance_matrix.shape}."
        )
    if block_h <= 0 or block_w <= 0:
        raise ValueError("block_h and block_w must be > 0.")

    src = distance_matrix.astype(np.float64, copy=False)
    h, w = src.shape
    out_h = (h + block_h - 1) // block_h
    out_w = (w + block_w - 1) // block_w
    out = np.full((out_h, out_w), np.nan, dtype=np.float64)

    for i in range(out_h):
        r0 = i * block_h
        r1 = min((i + 1) * block_h, h)
        for j in range(out_w):
            c0 = j * block_w
            c1 = min((j + 1) * block_w, w)
            block = src[r0:r1, c0:c1]
            valid = block[np.isfinite(block)]
            if valid.size > 0:
                out[i, j] = reducer(valid)

    return out


def reduce_stereo_camera_data_mean(
    distance_matrix: np.ndarray,
    block_h: int = 2,
    block_w: int = 2,
) -> np.ndarray:
    """Return block-wise mean downsample of distance matrix."""
    return _block_reduce_distance_matrix(
        distance_matrix, block_h, block_w, lambda v: float(np.mean(v))
    )


def reduce_stereo_camera_data_median(
    distance_matrix: np.ndarray,
    block_h: int = 2,
    block_w: int = 2,
) -> np.ndarray:
    """Return block-wise median downsample of distance matrix."""
    return _block_reduce_distance_matrix(
        distance_matrix, block_h, block_w, lambda v: float(np.median(v))
    )


def reduce_stereo_camera_data_min(
    distance_matrix: np.ndarray,
    block_h: int = 2,
    block_w: int = 2,
) -> np.ndarray:
    """Return block-wise minimum downsample of distance matrix."""
    return _block_reduce_distance_matrix(
        distance_matrix, block_h, block_w, lambda v: float(np.min(v))
    )


def reduce_stereo_camera_data_max(
    distance_matrix: np.ndarray,
    block_h: int = 2,
    block_w: int = 2,
) -> np.ndarray:
    """Return block-wise maximum downsample of distance matrix."""
    return _block_reduce_distance_matrix(
        distance_matrix, block_h, block_w, lambda v: float(np.max(v))
    )


if __name__ == "__main__":
    camera = StereoCameraSource()
    data = camera.get_stereo_camera_data()
    print(data)