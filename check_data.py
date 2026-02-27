"""Select and visualize one simulated record as three heatmaps."""

from __future__ import annotations

from pathlib import Path
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np

DATA_DIR = Path("data/simulated_data")


def _pick_file_cli(data_dir: Path) -> Path:
    files = sorted(data_dir.glob("*.npz"))
    if not files:
        raise FileNotFoundError(f"No .npz files found in: {data_dir}")

    print("Available files:")
    for idx, file_path in enumerate(files, start=1):
        print(f"{idx:3d}. {file_path.name}")

    selected = input("Choose file number to visualize: ").strip()
    selected_idx = int(selected) - 1
    if selected_idx < 0 or selected_idx >= len(files):
        raise ValueError("Invalid file number.")
    return files[selected_idx]


def _pick_file_dialog(data_dir: Path) -> Path | None:
    try:
        import tkinter as tk
        from tkinter import filedialog
    except Exception:
        return None

    root = tk.Tk()
    root.withdraw()
    selected = filedialog.askopenfilename(
        title="Choose simulated data file",
        initialdir=str(data_dir.resolve()) if data_dir.exists() else str(Path.cwd()),
        filetypes=[("NPZ files", "*.npz")],
    )
    root.destroy()
    if not selected:
        return None
    return Path(selected)


def pick_file(data_dir: Path) -> Path:
    from_dialog = _pick_file_dialog(data_dir)
    if from_dialog is not None:
        return from_dialog
    return _pick_file_cli(data_dir)


def load_record(npz_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    with np.load(npz_path, allow_pickle=False) as record:
        if "camera_matrix" not in record or "esp_matrix" not in record:
            raise KeyError("The file must contain 'camera_matrix' and 'esp_matrix'.")
        camera_matrix = record["camera_matrix"]
        esp_matrix = record["esp_matrix"]
    return camera_matrix, esp_matrix


def normalize_camera_for_display(camera_matrix: np.ndarray) -> np.ndarray:
    camera = np.squeeze(camera_matrix)
    if camera.ndim == 2:
        return camera
    if camera.ndim >= 3:
        # Use first channel if camera has additional dimensions/channels.
        return camera[..., 0]
    raise ValueError(f"Unsupported camera shape for heatmap: {camera_matrix.shape}")


def compute_iq_maps(esp_iq: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if esp_iq.ndim != 3 or esp_iq.shape[-1] != 2:
        raise ValueError(
            "ESP IQ data must have shape (rows, cols, 2), "
            f"but got {esp_iq.shape}."
        )
    i_part = esp_iq[..., 0]
    q_part = esp_iq[..., 1]
    amplitude = np.sqrt(i_part**2 + q_part**2)
    phase = np.arctan2(q_part, i_part)
    return phase, amplitude


def show_heatmaps(phase: np.ndarray, amplitude: np.ndarray, camera: np.ndarray) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle("Simulated data visualization", fontsize=13)

    phase_img = axes[0].imshow(phase, cmap="twilight", aspect="auto")
    axes[0].set_title("ESP phase [rad]")
    axes[0].set_xlabel("Y")
    axes[0].set_ylabel("X")
    fig.colorbar(phase_img, ax=axes[0], fraction=0.046, pad=0.04)

    amp_img = axes[1].imshow(amplitude, cmap="viridis", aspect="auto")
    axes[1].set_title("ESP amplitude")
    axes[1].set_xlabel("Y")
    axes[1].set_ylabel("X")
    fig.colorbar(amp_img, ax=axes[1], fraction=0.046, pad=0.04)

    cam_img = axes[2].imshow(camera, cmap="magma", aspect="auto")
    axes[2].set_title("Camera matrix")
    axes[2].set_xlabel("Y")
    axes[2].set_ylabel("X")
    fig.colorbar(cam_img, ax=axes[2], fraction=0.046, pad=0.04)

    fig.tight_layout()
    plt.show()


def main() -> None:
    selected_file = pick_file(DATA_DIR)
    print(f"Selected file: {selected_file}")

    camera_matrix, esp_iq = load_record(selected_file)
    phase, amplitude = compute_iq_maps(esp_iq)
    camera_for_display = normalize_camera_for_display(camera_matrix)

    show_heatmaps(phase=phase, amplitude=amplitude, camera=camera_for_display)


if __name__ == "__main__":
    main()
