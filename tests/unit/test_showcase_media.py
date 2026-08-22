from __future__ import annotations

from pathlib import Path

from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SHOWCASE_DIRECTORY = PROJECT_ROOT / "docs" / "assets" / "showcase"
SCREENSHOTS = (
    "01-control-disconnected.png",
    "02-control-connected.png",
    "03-control-driving.png",
    "04-lidar-monitoring.png",
    "05-lidar-assisted-stop.png",
    "06-diagnostics.png",
)


def test_showcase_media_is_complete_readable_and_github_friendly() -> None:
    gif_path = SHOWCASE_DIRECTORY / "app-walkthrough.gif"
    assert gif_path.stat().st_size < 6 * 1024 * 1024
    with Image.open(gif_path) as walkthrough:
        assert walkthrough.format == "GIF"
        assert walkthrough.size == (960, 640)
        assert walkthrough.n_frames >= 20
        assert walkthrough.info.get("loop") == 0
        durations: list[int] = []
        for frame_index in range(walkthrough.n_frames):
            walkthrough.seek(frame_index)
            durations.append(int(walkthrough.info.get("duration", 0)))
        assert 8_000 <= sum(durations) <= 15_000

    for filename in SCREENSHOTS:
        screenshot_path = SHOWCASE_DIRECTORY / filename
        assert screenshot_path.stat().st_size < 700 * 1024
        with Image.open(screenshot_path) as screenshot:
            assert screenshot.format == "PNG"
            assert screenshot.size == (1200, 800)
            screenshot.verify()
