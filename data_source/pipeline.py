"""
DepthAI v3 pipeline builder for the OAK-D Lite FF.

Constructs a pipeline with:

- **Left / right mono cameras** (CAM_B, CAM_C) feeding a ``StereoDepth`` node.
- **Stereo depth** aligned to the RGB camera perspective (CAM_A) so depth
  pixels correspond 1:1 with RGB pixels.
- **RGB camera** (CAM_A) for the colour preview.
- **Calibration HFOV** read from the device and exposed for host-side
  spatial calculation.

Follows the official Luxonis v3 pattern::

    device   = dai.Device()
    pipeline = dai.Pipeline(device)
    # … build nodes …
    pipeline.start()

Reference examples
------------------
- ``oak-examples/depth-measurement/calc-spatial-on-host``
- ``oak-examples/tutorials/camera-stereo-depth``
"""

from __future__ import annotations

from dataclasses import dataclass

import depthai as dai

# ---------------------------------------------------------------------------
# Configuration dataclass
# ---------------------------------------------------------------------------


@dataclass
class PipelineConfig:
    """Tuneable parameters for the stereo-depth + RGB pipeline.

    Attributes
    ----------
    stereo_resolution
        ``(width, height)`` of each mono camera feed.
    stereo_fps
        Frame rate cap for mono cameras (``None`` = device default).
    stereo_preset
        ``StereoDepth.PresetMode`` — e.g. ``DEFAULT``, ``DENSITY``,
        ``ACCURACY``.
    left_right_check
        Enable left-right consistency check (removes "ghost" objects).
    subpixel
        Enable sub-pixel disparity for smoother depth at long range.
    extended_disparity
        Enable extended disparity for better close-range depth.
    depth_lower_threshold_mm, depth_upper_threshold_mm
        Valid depth range used by the host-side spatial calculation.
    rgb_resolution
        ``(width, height)`` of the RGB camera output.  Must have a
        width that is a multiple of 16 (StereoDepth requirement).
    """

    stereo_resolution: tuple[int, int] = (640, 400)
    stereo_fps: float | None = None
    stereo_preset: dai.node.StereoDepth.PresetMode = (
        dai.node.StereoDepth.PresetMode.DEFAULT
    )
    left_right_check: bool = True
    subpixel: bool = True
    extended_disparity: bool = False

    depth_lower_threshold_mm: int = 200
    depth_upper_threshold_mm: int = 10_000

    rgb_resolution: tuple[int, int] = (640, 400)


# ---------------------------------------------------------------------------
# Pipeline wrapper
# ---------------------------------------------------------------------------


class CameraPipeline:
    """High-level wrapper around the DepthAI v3 pipeline.

    Typical usage::

        config = PipelineConfig()
        with CameraPipeline(config) as pipe:
            while pipe.is_running():
                rgb_msg   = pipe.q_rgb.tryGet()
                depth_msg = pipe.q_depth.tryGet()
                ...

    The context manager calls :meth:`start` / :meth:`stop` automatically.
    """

    def __init__(self, config: PipelineConfig | None = None) -> None:
        self.config = config or PipelineConfig()

        # Device & pipeline — created in start()
        self._device: dai.Device | None = None
        self._pipeline: dai.Pipeline | None = None

        # Nodes — populated by _build_pipeline()
        self._stereo: dai.node.StereoDepth | None = None
        self._rgb_cam: dai.node.Camera | None = None
        self._left_cam: dai.node.Camera | None = None
        self._right_cam: dai.node.Camera | None = None

        # Output queues — populated by _build_pipeline()
        self.q_depth: dai.MessageQueue | None = None
        self.q_rgb: dai.MessageQueue | None = None

        # Calibration — populated by _build_pipeline()
        self.hfov_deg: float = 0.0
        self._calib: dai.CalibrationHandler | None = None

    # ------------------------------------------------------------------
    # Pipeline construction (internal)
    # ------------------------------------------------------------------

    def _build_pipeline(self) -> None:
        """Create camera, stereo, and output-queue nodes.

        Must be called *after* :pyattr:`_device` and :pyattr:`_pipeline`
        have been initialised.
        """
        assert self._pipeline is not None
        assert self._device is not None

        cfg = self.config
        pipeline = self._pipeline

        # ---- Left / right mono cameras ----
        self._left_cam = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B,
        )
        self._right_cam = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C,
        )

        left_out = self._left_cam.requestOutput(
            cfg.stereo_resolution, type=dai.ImgFrame.Type.NV12
        )
        right_out = self._right_cam.requestOutput(
            cfg.stereo_resolution, type=dai.ImgFrame.Type.NV12
        )

        # ---- Stereo depth ----
        self._stereo = pipeline.create(dai.node.StereoDepth).build(
            left=left_out,
            right=right_out,
            presetMode=cfg.stereo_preset,
        )
        self._stereo.setLeftRightCheck(cfg.left_right_check)
        self._stereo.setSubpixel(cfg.subpixel)
        if cfg.extended_disparity:
            self._stereo.setExtendedDisparity(True)

        # Align depth to RGB so each depth pixel maps to the same RGB pixel.
        self._stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        self._stereo.setOutputSize(cfg.rgb_resolution[0], cfg.rgb_resolution[1])

        # ---- RGB camera ----
        self._rgb_cam = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A,
        )

        # ---- Output queues ----
        self.q_depth = self._stereo.depth.createOutputQueue()
        rgb_output = self._rgb_cam.requestOutput(cfg.rgb_resolution)
        self.q_rgb = rgb_output.createOutputQueue()

        # ---- Read calibration (needed for host-side spatial calc) ----
        self._calib = self._device.readCalibration()
        self.hfov_deg = self._calib.getFov(
            dai.CameraBoardSocket.CAM_A,
            useSpec=False,
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Connect to the OAK-D device and start streaming.

        Execution order follows the v3 pattern:
        ``Device()`` → ``Pipeline(device)`` → build nodes → ``start()``.
        """
        self._device = dai.Device()
        self._pipeline = dai.Pipeline(self._device)
        self._build_pipeline()
        self._pipeline.start()

    def stop(self) -> None:
        """Stop the pipeline and release device resources."""
        if self._pipeline is not None and self._pipeline.isRunning():
            self._pipeline.stop()
        self._pipeline = None
        self._device = None

    def is_running(self) -> bool:
        """Return ``True`` while the pipeline is actively streaming."""
        return self._pipeline is not None and self._pipeline.isRunning()

    # ------------------------------------------------------------------
    # Context-manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> CameraPipeline:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()
