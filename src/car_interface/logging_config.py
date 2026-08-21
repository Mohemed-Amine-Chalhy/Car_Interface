"""Application logging and local diagnostic paths."""

from __future__ import annotations

import logging
import os
import time
from logging.handlers import RotatingFileHandler
from pathlib import Path


def application_data_directory() -> Path:
    """Return the per-user writable application directory."""

    if os.name == "nt":
        base = Path(os.environ.get("LOCALAPPDATA", Path.home() / "AppData" / "Local"))
    else:
        base = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state"))
    return base / "CarInterface"


def configure_logging(level: str = "INFO", *, log_directory: Path | None = None) -> Path:
    """Configure console and bounded rotating-file logging."""

    directory = application_data_directory() / "logs" if log_directory is None else log_directory
    directory.mkdir(parents=True, exist_ok=True)
    log_path = directory / "car-interface.log"

    root = logging.getLogger()
    root.setLevel(level.upper())
    for handler in tuple(root.handlers):
        root.removeHandler(handler)

    formatter = logging.Formatter(
        "%(asctime)s.%(msecs)03dZ %(levelname)s %(name)s %(threadName)s %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S",
    )
    formatter.converter = time.gmtime
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    file_handler = RotatingFileHandler(
        log_path,
        maxBytes=2_000_000,
        backupCount=5,
        encoding="utf-8",
    )
    file_handler.setFormatter(formatter)
    root.addHandler(console)
    root.addHandler(file_handler)
    logging.captureWarnings(True)
    return log_path
