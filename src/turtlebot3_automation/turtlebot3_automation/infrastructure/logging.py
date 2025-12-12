"""Logging utilities for the TurtleBot3 automation stack."""

from __future__ import annotations

import logging
import os
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Optional

_DEFAULT_LOG_DIR = Path(os.getenv("ROS_LOG_DIR", Path.home() / ".ros" / "turtlebot3_automation"))
_DEFAULT_LOG_DIR.mkdir(parents=True, exist_ok=True)


def configure_logging(name: str, log_file: Optional[Path] = None, console: bool = True) -> logging.Logger:
    """Configure a named logger with rotating-file handler and optional console handler."""
    logger = logging.getLogger(name)
    if logger.handlers:  # already configured
        return logger

    logger.setLevel(logging.INFO)
    logger.propagate = False  # Prevent propagation to root logger to avoid double printing

    if console:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(logging.Formatter("[%(levelname)s] %(name)s: %(message)s"))
        logger.addHandler(console_handler)

    if log_file:
        log_file = Path(log_file)
        log_file.parent.mkdir(parents=True, exist_ok=True)
        file_handler = RotatingFileHandler(log_file, maxBytes=2 * 1024 * 1024, backupCount=5)
        file_handler.setFormatter(
            logging.Formatter("%(asctime)s %(levelname)s %(name)s - %(message)s")
        )
        logger.addHandler(file_handler)

    if console:
        logger.debug("Logger %s configured with log file %s", name, log_file)
    return logger
