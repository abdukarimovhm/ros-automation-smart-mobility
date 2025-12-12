"""Command line interface for TurtleBot3 automation workflows."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path
from typing import List

from .setup_automation.installer import (
    automate_full_setup,
    build_workspace,
    configure_bashrc,
    configure_workspace,
    install_python_packages,
    install_ros_packages,
    sync_package_into_workspace,
)
from .infrastructure.logging import configure_logging

_LOG = configure_logging(__name__)


def _ros2_launch(package: str, launch_file: str, extra_args: List[str] | None = None) -> None:
    command = ["ros2", "launch", package, launch_file]
    if extra_args:
        command.extend(extra_args)
    _LOG.info("Executing: %s", " ".join(command))
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as exc:
        _LOG.error("Launch failed with exit code %s", exc.returncode)
        sys.exit(exc.returncode)
    except KeyboardInterrupt:
        _LOG.info("Launch interrupted by user.")
        sys.exit(130)


def _ros2_run(package: str, executable: str, args: List[str] | None = None) -> None:
    command = ["ros2", "run", package, executable]
    if args:
        command.extend(args)
    _LOG.info("Executing: %s", " ".join(command))
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as exc:
        _LOG.error("Node execution failed with exit code %s", exc.returncode)
        sys.exit(exc.returncode)
    except KeyboardInterrupt:
        _LOG.info("Node execution interrupted by user.")
        sys.exit(130)


def _default_workspace() -> Path:
    return Path.home() / "turtlebot3_automation_ws"


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=_default_workspace(),
        help="colcon workspace location (default: %(default)s)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Plan actions without executing system changes.",
    )

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("setup", help="Run the full automation provisioning pipeline.")

    ros_only = sub.add_parser("install-ros", help="Install ROS 2 Humble and TurtleBot3 packages via apt.")
    ros_only.add_argument("--python", default=sys.executable, help="Python executable for pip installs.")

    workspace = sub.add_parser("workspace", help="Create and populate the colcon workspace.")
    workspace.add_argument(
        "--sync",
        action="store_true",
        help="Symlink the local package into the workspace after creating it.",
    )

    sub.add_parser("build", help="Run colcon build inside the workspace.")

    maintenance = sub.add_parser("maintenance", help="Launch the maintenance monitoring node.")
    maintenance.add_argument(
        "--as-node",
        action="store_true",
        help="Run the Python node directly instead of using the launch description.",
    )

    navigation = sub.add_parser("navigation", help="Launch navigation automation components.")
    navigation.add_argument(
        "--mode",
        choices=["slam", "map"],
        default="slam",
        help="Whether to start SLAM or map localization mode.",
    )
    navigation.add_argument(
        "--map-yaml",
        type=str,
        default="",
        help="Map YAML file when running in map mode.",
    )

    detection = sub.add_parser("detect", help="Launch YOLO-based perception pipeline.")
    detection.add_argument(
        "--camera-topic",
        default="/camera/image_raw",
        help="Image topic to subscribe to.",
    )

    sub.add_parser("qr-follow", help="Start the QR-code following custom feature.")

    return parser.parse_args(argv)


def main(argv: List[str] | None = None) -> None:
    args = parse_args(argv or sys.argv[1:])
    workspace = args.workspace
    dry_run: bool = getattr(args, "dry_run", False)

    try:
        if args.command == "setup":
            automate_full_setup(workspace, dry_run=dry_run)
            return

        if args.command == "install-ros":
            install_ros_packages(dry_run=dry_run)
            install_python_packages(dry_run=dry_run, python_executable=args.python)
            return

        if args.command == "workspace":
            configure_workspace(workspace, dry_run=dry_run)
            if args.sync:
                package_root = Path(__file__).resolve().parents[1]
                sync_package_into_workspace(package_root, workspace, dry_run=dry_run)
            else:
                _LOG.info("Skipping package sync (use --sync to enable).")
            configure_bashrc(workspace, dry_run=dry_run)
            return

        if args.command == "build":
            build_workspace(workspace, dry_run=dry_run)
            return

        if args.command == "maintenance":
            if args.as_node:
                _ros2_run("turtlebot3_automation", "maintenance_monitor", [])
            else:
                _ros2_launch("turtlebot3_automation", "maintenance.launch.py", [])
            return

        if args.command == "navigation":
            extra: List[str] = [f"mode:={args.mode}"]
            if args.map_yaml:
                extra.append(f"map_yaml:={args.map_yaml}")
            _ros2_launch("turtlebot3_automation", "navigation_only.launch.py", extra)
            return

        if args.command == "detect":
            extra = [f"camera_topic:={args.camera_topic}"]
            _ros2_launch("turtlebot3_automation", "object_detection.launch.py", extra)
            return

        if args.command == "qr-follow":
            _ros2_launch("turtlebot3_automation", "qr_follow.launch.py", [])
            return

    except subprocess.CalledProcessError as exc:
        _LOG.error("Command failed with exit code %s", exc.returncode)
        sys.exit(exc.returncode)
    except KeyboardInterrupt:
        _LOG.info("Operation interrupted by user.")
        sys.exit(130)
    except Exception as exc:
        _LOG.exception("Unexpected error: %s", exc)
        sys.exit(1)

    raise ValueError(f"Unhandled command {args.command}")


if __name__ == "__main__":  # pragma: no cover
    main()
