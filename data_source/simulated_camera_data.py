"""Utilities for generating simulated camera data."""

from __future__ import annotations

from typing import Optional

import numpy as np


def generate_simulated_camera_data(
    rows: int,
    cols: int,
    value_min: float = 0.0,
    value_max: float = 1.0,
    dtype: str = "float32",
    seed: Optional[int] = None,
) -> np.ndarray:
    """
    Generate one simulated camera matrix.

    Args:
        rows: Number of matrix rows (X dimension).
        cols: Number of matrix columns (Y dimension).
        value_min: Minimum random value (inclusive).
        value_max: Maximum random value (exclusive).
        dtype: Output numpy dtype, e.g. "float32", "float64", "int32".
        seed: Optional random seed for reproducible output.
    """
    if rows <= 0 or cols <= 0:
        raise ValueError("rows and cols must be greater than 0.")
    if value_min >= value_max:
        raise ValueError("value_min must be smaller than value_max.")

    rng = np.random.default_rng(seed)
    target_dtype = np.dtype(dtype)

    if np.issubdtype(target_dtype, np.integer):
        low = int(np.floor(value_min))
        high = int(np.ceil(value_max))
        if low >= high:
            raise ValueError(
                "For integer dtype, rounded bounds must satisfy low < high."
            )
        return rng.integers(low=low, high=high, size=(rows, cols), dtype=target_dtype)

    data = rng.uniform(low=value_min, high=value_max, size=(rows, cols))
    return data.astype(target_dtype, copy=False)
