"""Utilities for generating simulated ESP sensor data."""

from __future__ import annotations

from typing import Optional

import numpy as np


def generate_simulated_esp_data(
    rows: int,
    cols: int,
    value_min: float = -1.0,
    value_max: float = 1.0,
    dtype: str = "float32",
    seed: Optional[int] = None,
) -> np.ndarray:
    """
    Generate simulated ESP IQ samples.

    Args:
        rows: Number of IQ sample rows (X dimension).
        cols: Number of IQ sample columns (Y dimension).
        value_min: Minimum random value (inclusive).
        value_max: Maximum random value (exclusive).
        dtype: Output numpy dtype, e.g. "float32", "float64", "int16".
        seed: Optional random seed for reproducible output.

    Returns:
        Numpy array with shape (rows, cols, 2):
        [..., 0] -> I component
        [..., 1] -> Q component
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
        i_data = rng.integers(low=low, high=high, size=(rows, cols), dtype=target_dtype)
        q_data = rng.integers(low=low, high=high, size=(rows, cols), dtype=target_dtype)
        return np.stack((i_data, q_data), axis=-1)

    i_data = rng.uniform(low=value_min, high=value_max, size=(rows, cols))
    q_data = rng.uniform(low=value_min, high=value_max, size=(rows, cols))
    iq_data = np.stack((i_data, q_data), axis=-1)
    return iq_data.astype(target_dtype, copy=False)
