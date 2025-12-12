"""TurtleBot3 automation toolkit for ROS 2 Humble."""

from importlib.metadata import version, PackageNotFoundError

try:  # pragma: no cover - best effort metadata lookup
    __version__ = version("turtlebot3_automation")
except PackageNotFoundError:  # pragma: no cover - local development fallback
    __version__ = "0.1.0"

__all__ = ["__version__"]
