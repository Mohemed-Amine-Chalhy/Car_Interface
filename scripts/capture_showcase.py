"""Capture the real simulator walkthrough as optimized repository media."""

from __future__ import annotations

import argparse
import ctypes
import os
import tkinter as tk
from pathlib import Path
from typing import ClassVar

from PIL import Image, ImageGrab

from car_interface.config import AppConfig
from car_interface.factory import build_control_service
from car_interface.services.control import ControlService
from car_interface.ui import CarInterfaceWindow, ShowcaseDirector

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "docs" / "assets" / "showcase"
FRAME_INTERVAL_MS = 125
GIF_WIDTH = 960
GIF_COLORS = 128


def _enable_windows_dpi_awareness() -> None:
    if os.name != "nt":
        return
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(2)
    except (AttributeError, OSError):
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except (AttributeError, OSError):
            return


class ShowcaseCapture:
    """Collect GIF frames and named screenshots from one visible Tk window."""

    SCREENSHOT_STAGES: ClassVar[dict[str, str]] = {
        "connected": "02-control-connected.png",
        "driving": "03-control-driving.png",
        "lidar_monitoring": "04-lidar-monitoring.png",
        "lidar_assisted_stop": "05-lidar-assisted-stop.png",
        "diagnostics": "06-diagnostics.png",
    }

    def __init__(
        self,
        root: tk.Tk,
        window: CarInterfaceWindow,
        service: ControlService,
        output_directory: Path,
    ) -> None:
        self.root = root
        self.window = window
        self.output_directory = output_directory
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self.frames: list[Image.Image] = []
        self._capturing = True
        self.director = ShowcaseDirector(
            root,
            window,
            service,
            on_stage=self._on_stage,
            on_complete=self._schedule_completion,
        )

    def start(self) -> None:
        self.root.update_idletasks()
        self._save_screenshot("01-control-disconnected.png")
        self._capture_frame()
        self.root.after(700, self.director.start)

    def _on_stage(self, stage: str) -> None:
        filename = self.SCREENSHOT_STAGES.get(stage)
        if filename is not None:
            self.root.after(300, lambda: self._save_screenshot(filename))

    def _schedule_completion(self) -> None:
        self.root.after(700, self._complete)

    def _capture_frame(self) -> None:
        if not self._capturing:
            return
        self.frames.append(self._resize_for_gif(self._grab_window()))
        self.root.after(FRAME_INTERVAL_MS, self._capture_frame)

    def _complete(self) -> None:
        if not self._capturing:
            return
        self.frames.append(self._resize_for_gif(self._grab_window()))
        self._capturing = False
        self.root.quit()

    def _save_screenshot(self, filename: str) -> None:
        image = self._grab_window()
        image.save(self.output_directory / filename, format="PNG", optimize=True)

    def _grab_window(self) -> Image.Image:
        self.root.update_idletasks()
        try:
            image = ImageGrab.grab(
                window=self.root.winfo_id(),
                include_layered_windows=True,
            )
        except (OSError, TypeError):
            left = self.root.winfo_rootx()
            top = self.root.winfo_rooty()
            right = left + self.root.winfo_width()
            bottom = top + self.root.winfo_height()
            image = ImageGrab.grab(
                bbox=(left, top, right, bottom),
                include_layered_windows=True,
            )
        return image.convert("RGB")

    @staticmethod
    def _resize_for_gif(image: Image.Image) -> Image.Image:
        height = round(image.height * GIF_WIDTH / image.width)
        return image.resize((GIF_WIDTH, height), Image.Resampling.LANCZOS)

    def write_gif(self) -> Path:
        if not self.frames:
            raise RuntimeError("the walkthrough completed without capturing any frames")
        destination = self.output_directory / "app-walkthrough.gif"
        palette_frames = [
            frame.quantize(colors=GIF_COLORS, method=Image.Quantize.MEDIANCUT)
            for frame in self.frames
        ]
        palette_frames[0].save(
            destination,
            save_all=True,
            append_images=palette_frames[1:],
            duration=FRAME_INTERVAL_MS,
            loop=0,
            optimize=True,
            disposal=1,
        )
        return destination


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="directory for the GIF and numbered PNG screenshots",
    )
    return parser


def main(arguments: list[str] | None = None) -> int:
    namespace = _parser().parse_args(arguments)
    _enable_windows_dpi_awareness()
    config = AppConfig(mode="simulation")
    service = build_control_service(config)
    root = tk.Tk()
    window = CarInterfaceWindow(
        root,
        service,
        simulation=True,
        max_speed_percent=config.max_speed_percent,
    )
    root.geometry("1200x800+40+40")
    root.update_idletasks()
    root.lift()
    root.attributes("-topmost", True)
    root.after(1_000, lambda: root.attributes("-topmost", False))
    capture = ShowcaseCapture(root, window, service, namespace.output_dir.resolve())
    try:
        root.after(350, capture.start)
        root.mainloop()
    finally:
        capture.director.stop()
        service.shutdown()
        root.destroy()
    gif_path = capture.write_gif()
    print(f"Captured {len(capture.frames)} frames: {gif_path}")
    for screenshot in sorted(namespace.output_dir.resolve().glob("*.png")):
        print(f"Screenshot: {screenshot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
