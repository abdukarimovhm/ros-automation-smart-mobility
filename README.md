# TurtleBot3 Automation Suite (ROS 2 Foxy)

Comprehensive automation toolkit for TurtleBot3 on ROS 2 Foxy (Ubuntu 20.04). The suite provides autonomous navigation, YOLOv8 object detection, maintenance monitoring, and custom QR-following capabilities. Each feature is implemented as a modular ROS 2 node with unified CLI and launch file interfaces.

## Features

- **Navigation Automation** – Complete SLAM and map-based autonomous navigation using Nav2 stack
- **Object Detection** – Real-time YOLOv8 inference on camera streams with ROS 2 message publishing
- **Maintenance Monitoring** – Battery, diagnostics, and system health monitoring with alerts
- **Custom QR Follow** – QR code detection and following behavior for robot guidance

## Quick Start (From Zero)

### 1. Clone and Setup Environment

```bash
# Clone the repository
git clone https://github.com/BaratovSokhibjon/module.ros2-automation
cd module.ros2-automation

# Create conda environment with Python 3.8 (required for ROS Foxy)
conda create -n ros-foxy python=3.8 -y
conda activate ros-foxy

# Install Python dependencies
pip install -r requirements.txt
```

### 2. Install ROS Foxy System Packages

```bash
# Update package lists
sudo apt update

# Install ROS Foxy desktop and TurtleBot3 packages
sudo apt install -y ros-foxy-desktop ros-foxy-turtlebot3* ros-foxy-nav2* ros-foxy-slam-toolbox ros-foxy-vision-msgs

# Install additional build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

### 3. Build the Application

```bash
# Initialize rosdep
sudo rosdep init
rosdep update

# Build the package
colcon build --symlink-install
source install/setup.bash
```

### 4. Run the Application

#### Option A: Full Navigation Stack (Recommended)
```bash
# Launch complete navigation system with SLAM
ros2 launch turtlebot3_automation navigation_only.launch.py
```

#### Option B: Individual Components
```bash
# Maintenance monitoring only
ros2 launch turtlebot3_automation maintenance.launch.py

# Object detection only
ros2 launch turtlebot3_automation object_detection.launch.py

# QR following only
ros2 launch turtlebot3_automation qr_follow.launch.py
```

## Repository Layout

```
src/turtlebot3_automation/
├── turtlebot3_automation/        # Main Python package
│   ├── cli.py                     # Command-line interface
│   ├── maintenance/               # System monitoring node
│   ├── navigation/                # Navigation management
│   ├── perception/                # Object detection pipeline
│   └── custom_features/           # QR following feature
├── config/                        # YAML configuration files
├── launch/                        # ROS 2 launch files
├── tests/                         # Unit tests
└── package.xml / setup.py         # ROS 2 package metadata
requirements.txt                   # Python dependencies
yolov8n.pt                        # YOLOv8 model weights
```

## Prerequisites

- **Ubuntu 20.04** (Focal Fossa)
- **Python 3.8** (via conda environment)
- **ROS 2 Foxy** (system installation)
- **TurtleBot3 packages** (Gazebo simulation or hardware)
- **GPU optional** (YOLOv8 works on CPU)

## Usage

### Launch Files

- **Complete Navigation Stack** (SLAM + Navigation):
  ```bash
  ros2 launch turtlebot3_automation navigation_only.launch.py
  ```
  This launches the full Nav2 stack with SLAM Toolbox for mapping and autonomous navigation.

- **Maintenance Monitoring**:
  ```bash
  ros2 launch turtlebot3_automation maintenance.launch.py
  ```
  Monitors system health, battery, and diagnostics.

- **Object Detection**:
  ```bash
  ros2 launch turtlebot3_automation object_detection.launch.py
  ```
  Runs YOLOv8 inference on camera streams.

- **QR Code Following**:
  ```bash
  ros2 launch turtlebot3_automation qr_follow.launch.py
  ```
  Enables QR code detection and following behavior.

### Individual Nodes

```bash
# Run maintenance monitor
ros2 run turtlebot3_automation maintenance_monitor

# Run object detector
ros2 run turtlebot3_automation yolo_detector

# Run QR follower
ros2 run turtlebot3_automation qr_follow
```

## Configuration

### Maintenance Monitoring
- **Config file**: `src/turtlebot3_automation/config/maintenance.yaml`
- **Topics**:
  - Publishes status on `turtlebot3/maintenance/status`
  - Reports every 30 seconds with battery, diagnostics, and system health

### Object Detection
- **Config file**: `src/turtlebot3_automation/config/object_detection.yaml`
- **Model**: Uses YOLOv8n (`yolov8n.pt`) - downloads automatically on first run
- **Topics**:
  - Detections: `turtlebot3/perception/detections` (`Detection2DArray`)
  - Markers: `turtlebot3/perception/markers` (`MarkerArray`)
  - Labels: `turtlebot3/perception/labels` (`String`)

### Navigation
- **Config file**: `src/turtlebot3_automation/config/navigation_params.yaml`
- **Features**: SLAM mapping, autonomous waypoint navigation, obstacle avoidance
- **Components**: Nav2 stack with behavior trees, recovery behaviors, and lifecycle management

## Testing

Run parameter validation tests:
```bash
colcon test --packages-select turtlebot3_automation
```

## Troubleshooting

### Common Issues

1. **ROS Version Conflicts**: Ensure you're using ROS Foxy (not Rolling) with Python 3.8
2. **Conda Environment**: Always activate the `ros2` conda environment before running
3. **Build Issues**: Run `colcon build --symlink-install` after config changes
4. **Navigation Manager**: Requires `ros-foxy-nav2-simple-commander` (install with sudo if needed)

### Environment Setup Verification

```bash
# Check conda environment
conda activate ros-foxy
python --version  # Should be 3.8.x

# Check ROS
source /opt/ros/foxy/setup.bash
ros2 --version

# Check package
source install/setup.bash
ros2 pkg list | grep turtlebot3_automation
```
