"""Resource path helpers for the automation package."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory


def get_share_path(*relative: str, package: str = "turtlebot3_automation") -> Path:
    """Return an absolute path inside the package share directory."""
    base = Path(get_package_share_directory(package))
    return base.joinpath(*relative)


def resolve_path(path: str, fallback_dir: Optional[Path] = None) -> Path:
    """Resolve a user-provided path, expanding `~` and making relative paths absolute."""
    expanded = Path(path).expanduser()
    if expanded.is_absolute():
        return expanded
    if fallback_dir:
        return fallback_dir / expanded
    return Path.cwd() / expanded
