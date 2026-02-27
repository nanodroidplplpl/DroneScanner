"""Generate and save one simulated camera+ESP data record."""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

import numpy as np

from data_source.simulated_camera_data import generate_simulated_camera_data
from data_source.simulated_esp_data import generate_simulated_esp_data

# =========================
# Configuration parameters
# =========================
CAMERA_ROWS = 64
CAMERA_COLS = 64
CAMERA_MIN = 0.0
CAMERA_MAX = 255.0
CAMERA_DTYPE = "float32"
CAMERA_SEED = None

ESP_ROWS = 64
ESP_COLS = 100
ESP_MIN = -2.0
ESP_MAX = 2.0
ESP_DTYPE = "float32"
ESP_SEED = None

OUTPUT_DIR = Path("data/simulated_data")
RECORD_NAME = None  # Example: "sample_001". If None, timestamp name is used.
NUM_RECORDS = 10  # How many simulated records to generate.


def save_record(
    camera_matrix: np.ndarray,
    esp_matrix: np.ndarray,
    output_dir: Path,
    record_name: str | None = None,
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)

    if not record_name:
        record_name = datetime.now().strftime("record_%Y%m%d_%H%M%S_%f")

    output_path = output_dir / f"{record_name}.npz"
    metadata = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "camera_shape": list(camera_matrix.shape),
        "camera_dtype": str(camera_matrix.dtype),
        "esp_shape": list(esp_matrix.shape),
        "esp_dtype": str(esp_matrix.dtype),
    }

    np.savez_compressed(
        output_path,
        camera_matrix=camera_matrix,
        esp_matrix=esp_matrix,
        metadata_json=json.dumps(metadata),
    )
    return output_path


def main() -> None:
    if NUM_RECORDS <= 0:
        raise ValueError("NUM_RECORDS must be greater than 0.")

    for i in range(NUM_RECORDS):
        # If seed is set, offset by index so each record differs but stays reproducible.
        camera_seed = None if CAMERA_SEED is None else CAMERA_SEED + i
        esp_seed = None if ESP_SEED is None else ESP_SEED + i

        camera_matrix = generate_simulated_camera_data(
            rows=CAMERA_ROWS,
            cols=CAMERA_COLS,
            value_min=CAMERA_MIN,
            value_max=CAMERA_MAX,
            dtype=CAMERA_DTYPE,
            seed=camera_seed,
        )
        esp_matrix = generate_simulated_esp_data(
            rows=ESP_ROWS,
            cols=ESP_COLS,
            value_min=ESP_MIN,
            value_max=ESP_MAX,
            dtype=ESP_DTYPE,
            seed=esp_seed,
        )

        record_name = None if RECORD_NAME is None else f"{RECORD_NAME}_{i:05d}"
        saved_path = save_record(
            camera_matrix=camera_matrix,
            esp_matrix=esp_matrix,
            output_dir=OUTPUT_DIR,
            record_name=record_name,
        )
        print(f"[{i + 1}/{NUM_RECORDS}] Saved: {saved_path}")

    print("Done.")
    print("camera_shape=%s esp_shape=%s" % (camera_matrix.shape, esp_matrix.shape))


if __name__ == "__main__":
    main()
