"""Automation helpers for provisioning ROS 2 Humble and TurtleBot3 dependencies."""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable, List, Sequence

from ..infrastructure.logging import configure_logging

_LOG = configure_logging(__name__)

ROS_APT_PACKAGES = [
    "ros-humble-desktop",
    "ros-humble-turtlebot3-bringup",
    "ros-humble-turtlebot3-navigation",
    "ros-humble-turtlebot3-gazebo",
    "ros-humble-turtlebot3-slam",
    "ros-humble-nav2-bringup",
    "ros-humble-slam-toolbox",
    "ros-humble-vision-msgs",
    "ros-humble-image-transport",
    "ros-humble-diagnostic-updater",
]

PYTHON_PACKAGES = [
    "ultralytics==8.0.196",
    "opencv-python>=4.7",
    "numpy>=1.23",
    "pyyaml>=6.0",
    "pillow",
    "typer==0.9.0",
    "jinja2>=3.1",
]


def _run(
    command: Sequence[str], *, dry_run: bool = False, env: dict | None = None, cwd: Path | None = None
) -> None:
    printable = " ".join(shlex.quote(part) for part in command)
    if dry_run:
        _LOG.info("[dry-run] (cwd=%s) %s", cwd or os.getcwd(), printable)
        return
    _LOG.info("Running: %s (cwd=%s)", printable, cwd or os.getcwd())
    subprocess.run(command, check=True, env=env, cwd=str(cwd) if cwd else None)


def ensure_apt_repositories(*, dry_run: bool = False) -> None:
    """Enable ROS 2 repositories and keys on Ubuntu 20.04."""
    distro = os.environ.get("ROSDISTRO", "humble")
    if distro != "humble":
        _LOG.warning("Detected ROSDISTRO=%s, expected foxy. Continuing anyway.", distro)

    if shutil.which("ros2"):
        _LOG.info("ros2 already present on PATH; apt repository setup skipped.")
        return

    _LOG.info("Configuring ROS 2 apt repositories for Ubuntu Focal.")
    sudo = shutil.which("sudo")
    prefix: List[str] = [sudo] if sudo else []

    _run(prefix + ["apt", "update"], dry_run=dry_run)
    _run(
        prefix
        + [
            "apt", "install", "-y",
            "curl",
            "gnupg2",
            "lsb-release",
        ],
        dry_run=dry_run,
    )

    keyring = Path("/usr/share/keyrings/ros-archive-keyring.gpg")
    if not keyring.exists():
        _run(
            [
                "sudo", "bash", "-c",
                "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc > /usr/share/keyrings/ros-archive-keyring.gpg",
            ],
            dry_run=dry_run,
        )

    repo_line = (
        "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] "
        "http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
    )
    sources_list = Path("/etc/apt/sources.list.d/ros2.list")
    if not sources_list.exists():
        _run(
            [
                "sudo",
                "bash",
                "-c",
                f"echo '{repo_line}' > /etc/apt/sources.list.d/ros2.list",
            ],
            dry_run=dry_run,
        )

    _run(prefix + ["apt", "update"], dry_run=dry_run)


def install_ros_packages(*, dry_run: bool = False) -> None:
    """Install the curated list of ROS 2 and TurtleBot3 packages via apt."""
    if shutil.which("ros2"):
        _LOG.info("ros2 CLI detected; ensuring TurtleBot3-specific packages are present.")
    ensure_apt_repositories(dry_run=dry_run)

    sudo = shutil.which("sudo")
    cmd = ([sudo] if sudo else []) + ["apt", "install", "-y"] + ROS_APT_PACKAGES
    _run(cmd, dry_run=dry_run)


def install_python_packages(*, dry_run: bool = False, python_executable: str = sys.executable) -> None:
    """Install the Python packages needed for the perception stack."""
    pip_cmd = [python_executable, "-m", "pip", "install", "--upgrade"] + PYTHON_PACKAGES
    _run(pip_cmd, dry_run=dry_run)


def configure_bashrc(workspace_path: Path, *, dry_run: bool = False) -> None:
    """Append TurtleBot3 environment hooks to ~/.bashrc if missing."""
    bashrc = Path.home() / ".bashrc"
    hook_marker = "# --- turtlebot3_automation ---"
    lines = [
        hook_marker,
        f"source /opt/ros/foxy/setup.bash",
        f"source {workspace_path.expanduser()}/install/setup.bash",
        "export TURTLEBOT3_MODEL=burger",
        "# --- turtlebot3_automation --- end ---",
    ]

    if bashrc.exists() and hook_marker in bashrc.read_text():
        _LOG.info("~/.bashrc already contains automation environment hooks.")
        return

    text = "\n" + "\n".join(lines) + "\n"
    if dry_run:
        _LOG.info("[dry-run] Would append to %s:%s%s", bashrc, os.linesep, text)
        return

    with bashrc.open("a", encoding="utf-8") as handle:
        handle.write(text)
    _LOG.info("Appended TurtleBot3 automation environment configuration to %s", bashrc)


def configure_workspace(workspace_path: Path, *, dry_run: bool = False) -> None:
    """Create a `colcon` workspace scaffold if it does not already exist."""
    workspace_path = workspace_path.expanduser().resolve()
    src_dir = workspace_path / "src"
    if dry_run:
        _LOG.info("[dry-run] Would create workspace at %s", workspace_path)
        return

    src_dir.mkdir(parents=True, exist_ok=True)
    _LOG.info("Ensured workspace directories exist at %s", workspace_path)


def sync_package_into_workspace(source_package: Path, workspace_path: Path, *, dry_run: bool = False) -> None:
    """Symlink or copy the local package into the target workspace."""
    workspace_src = workspace_path.expanduser().resolve() / "src"
    workspace_src.mkdir(parents=True, exist_ok=True)
    target = workspace_src / source_package.name

    if target.exists():
        _LOG.info("Package already present in workspace at %s", target)
        return

    if dry_run:
        _LOG.info("[dry-run] Would symlink %s -> %s", target, source_package)
        return

    target.symlink_to(source_package.resolve())
    _LOG.info("Linked %s into workspace %s", target, workspace_path)


def build_workspace(workspace_path: Path, *, dry_run: bool = False) -> None:
    """Invoke `colcon build` inside the workspace."""
    workspace_path = workspace_path.expanduser().resolve()
    if dry_run:
        _LOG.info("[dry-run] Would run colcon build in %s", workspace_path)
        return

    env = os.environ | {"COLCON_DEFAULTS_FILE": ""}
    _run(["colcon", "build"], dry_run=False, env=env, cwd=workspace_path)
    _LOG.info("colcon build finished")


def automate_full_setup(
    workspace_path: Path,
    *,
    dry_run: bool = False,
    python_executable: str = sys.executable,
) -> None:
    """Execute the full environment provisioning pipeline."""
    ensure_apt_repositories(dry_run=dry_run)
    install_ros_packages(dry_run=dry_run)
    install_python_packages(dry_run=dry_run, python_executable=python_executable)
    configure_workspace(workspace_path, dry_run=dry_run)
    configure_bashrc(workspace_path, dry_run=dry_run)

    source_package = Path(__file__).resolve().parents[2]
    sync_package_into_workspace(source_package, workspace_path, dry_run=dry_run)
    if not dry_run:
        _LOG.info("Reminder: run 'colcon build' inside %s before sourcing setup.bash", workspace_path)


__all__ = [
    "automate_full_setup",
    "install_ros_packages",
    "install_python_packages",
    "configure_workspace",
    "configure_bashrc",
    "sync_package_into_workspace",
    "build_workspace",
]
