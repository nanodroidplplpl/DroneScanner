"""Capture one measurement from ESP and stereo camera sources."""

from __future__ import annotations

from datetime import datetime
from typing import Any, Dict

from data_source.one_esp_control import get_one_eps_control_data
from data_source.stereo_camera import get_stereo_camera_data


def capture_data_one_time() -> Dict[str, Any]:
    """
    Capture one measurement package.

    Returns:
        Dictionary with timestamp, ESP data, and stereo camera data.
    """
    timestamp = datetime.now().isoformat(timespec="seconds")
    esp_data = get_one_eps_control_data()
    camera_data = get_stereo_camera_data()

    measurement = {
        "timestamp": timestamp,
        "esp_data": esp_data,
        "stereo_camera_data": camera_data,
    }
    return measurement


def main() -> None:
    measurement = capture_data_one_time()
    print("One measurement captured:")
    print(measurement)


if __name__ == "__main__":
    main()
