"""Basic validation of package parameter files."""

from __future__ import annotations

import yaml
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"


def _load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def test_navigation_params_has_waypoints() -> None:
    data = _load_yaml(CONFIG_DIR / "navigation_params.yaml")
    params = data["turtlebot3_automation"]["ros__parameters"]
    assert params["waypoints"], "Waypoints list must not be empty"


def test_detection_params_threshold() -> None:
    data = _load_yaml(CONFIG_DIR / "object_detection.yaml")
    params = data["turtlebot3_automation"]["ros__parameters"]
    assert 0.0 < params["confidence_threshold"] < 1.0
