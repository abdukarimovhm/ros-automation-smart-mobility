# TurtleBot3 Automation Suite (ROS 2 Foxy)

Comprehensive automation toolkit for TurtleBot3 on ROS 2 Foxy (Ubuntu 20.04). The suite provides autonomous navigation, YOLOv8 object detection, maintenance monitoring, and custom QR-following capabilities. Each feature is implemented as a modular ROS 2 node with unified CLI and launch file interfaces.

## Features

- **Navigation Automation** – Complete SLAM and map-based autonomous navigation using Nav2 stack
- **Object Detection** – Real-time YOLOv8 inference on camera streams with ROS 2 message publishing
- **Maintenance Monitoring** – Battery, diagnostics, and system health monitoring with alerts
- **Custom QR Follow** – QR code detection and following behavior for robot guidance
- **Test Scripts** – Simple bash scripts for testing nodes without hardware

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

#### Full Navigation Stack (With Hardware)
```bash
# Launch complete navigation system with SLAM
ros2 launch turtlebot3_automation navigation_only.launch.py
```

#### Individual Components
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
│   ├── custom_features/           # QR following feature
│   ├── setup_automation/          # Installation helpers
│   └── utils/                     # Shared utilities
├── config/                        # YAML configuration files
├── launch/                        # ROS 2 launch files
│   ├── navigation_only.launch.py  # Full navigation stack
│   ├── maintenance.launch.py      # Monitoring only
│   ├── object_detection.launch.py # Detection only
│   └── qr_follow.launch.py        # QR following only
├── tests/                         # Unit tests
└── package.xml / setup.py         # ROS 2 package metadata
scripts/                           # Test scripts
├── test_maintenance.sh            # Test battery monitoring
├── test_detection.sh              # Test object detection
└── test_qr_follow.sh              # Test QR following
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

### Testing Without Hardware

Three bash scripts are provided for testing each subsystem without physical robot hardware:

#### Test Maintenance Monitor
```bash
# Simulate battery drain and health monitoring
./scripts/test_maintenance.sh
```
Publishes simulated battery data that drains from 12.4V to 10.5V, triggering maintenance alerts.

#### Test Object Detection
```bash
# Simulate YOLO detections
./scripts/test_detection.sh
```
Publishes random object detection labels (person, car, dog, etc.) to test detection node output.

#### Test QR Follow
```bash
# Simulate robot movement
./scripts/test_qr_follow.sh
```
Publishes various movement commands to test QR follow node behavior.

**Usage pattern:**
1. In terminal 1: Launch the node you want to test
   ```bash
   ros2 launch turtlebot3_automation maintenance.launch.py
   ```
2. In terminal 2: Run the corresponding test script
   ```bash
   ./scripts/test_maintenance.sh
   ```
3. Observe the node's output in terminal 1

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

# Run navigation manager
ros2 run turtlebot3_automation navigation_manager
```

## Configuration

### Maintenance Monitoring
- **Config file**: `src/turtlebot3_automation/config/maintenance.yaml`
- **Topics**:
  - Publishes alerts on `turtlebot3/alerts`
  - Reports every 30 seconds with battery, diagnostics, and system health
  - Monitors `/battery_state`, `/diagnostics`, `/cmd_vel`

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

### Test Scripts (No Hardware Required)

Use the provided bash scripts to test nodes without physical hardware:

```bash
# Test maintenance monitoring
./scripts/test_maintenance.sh

# Test object detection
./scripts/test_detection.sh

# Test QR following
./scripts/test_qr_follow.sh
```

Each script publishes simulated data to the corresponding ROS 2 topics.

### Monitor Topics
```bash
# View battery status
ros2 topic echo /battery_state

# View object detections
ros2 topic echo turtlebot3/perception/labels

# View maintenance alerts
ros2 topic echo turtlebot3/alerts

# View robot velocity
ros2 topic echo /cmd_vel

# Check topic publishing rates
ros2 topic hz /battery_state
ros2 topic hz turtlebot3/perception/detections
```

### Visualize in RViz
```bash
# Terminal 1: Run nodes
ros2 launch turtlebot3_automation object_detection.launch.py

# Terminal 2: Launch RViz
rviz2
```
In RViz:
1. Set Fixed Frame to `base_link`
2. Add → By topic → `/turtlebot3/perception/markers` → MarkerArray

### Run Unit Tests
Run parameter validation tests:
```bash
colcon test --packages-select turtlebot3_automation
```

## Troubleshooting

### Common Issues

1. **ROS Version Conflicts**: Ensure you're using ROS Foxy (not Rolling) with Python 3.8
2. **Conda Environment**: Always activate the `ros-foxy` conda environment before running
3. **Build Issues**: Run `colcon build --symlink-install` after config changes
4. **Navigation Manager**: Requires `ros-foxy-nav2-simple-commander` (install with sudo if needed)

### Test Script Issues

**Scripts not executable:**
```bash
chmod +x scripts/*.sh
```

**No topic data appearing:**
- Verify the node is running: `ros2 node list`
- Check topic exists: `ros2 topic list`
- Verify topic data: `ros2 topic echo /battery_state`

**Camera topics not available:**
- For object detection and QR follow: Verify camera is publishing
- Check: `ros2 topic hz /camera/color/image_raw`
- If testing without hardware, camera topics won't be available

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

# Verify all executables are available
ros2 pkg executables turtlebot3_automation
```

## Demo Examples

### Example 1: Test Maintenance System
```bash
# Terminal 1: Launch maintenance node
ros2 launch turtlebot3_automation maintenance.launch.py

# Terminal 2: Run battery test
./scripts/test_maintenance.sh
```

### Example 2: Test Object Detection
```bash
# Terminal 1: Launch detection node
ros2 launch turtlebot3_automation object_detection.launch.py

# Terminal 2: Publish fake detections
./scripts/test_detection.sh
```

### Example 3: Monitor Multiple Systems
```bash
# Terminal 1: Run maintenance node
ros2 launch turtlebot3_automation maintenance.launch.py

# Terminal 2: Monitor battery
ros2 topic echo /battery_state

# Terminal 3: Monitor alerts
ros2 topic echo turtlebot3/alerts

# Terminal 4: Run test script
./scripts/test_maintenance.sh
```

## ROS 2 Topics

### Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/battery_state` | `sensor_msgs/BatteryState` | Battery voltage and percentage |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |
| `turtlebot3/alerts` | `std_msgs/String` | Maintenance alerts and warnings |
| `turtlebot3/perception/detections` | `vision_msgs/Detection2DArray` | Object detection results |
| `turtlebot3/perception/labels` | `std_msgs/String` | Human-readable detection labels |
| `turtlebot3/perception/markers` | `visualization_msgs/MarkerArray` | RViz markers for detections |

### Subscribed Topics

| Node | Subscribes To | Purpose |
|------|---------------|---------|
| Maintenance Monitor | `/battery_state`, `/diagnostics`, `/cmd_vel` | System health monitoring |
| Object Detection | `/camera/color/image_raw` | Image processing |
| QR Follow | `/camera/color/image_raw` | QR code detection |

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     TurtleBot3 Hardware/Simulation              │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐   │
│  │   Battery    │  │    Camera    │  │  Motor Controllers  │   │
│  └──────┬───────┘  └──────┬───────┘  └──────────┬──────────┘   │
└─────────┼──────────────────┼───────────────────────┼────────────┘
          │                  │                       │
          ▼                  ▼                       ▼
    /battery_state    /camera/image_raw         /cmd_vel
          │                  │                       ▲
          │                  │                       │
┌─────────┴──────────────────┴───────────────────────┴────────────┐
│                      ROS 2 Topic Layer                           │
└─────┬──────────┬──────────────┬─────────────────┬───────────────┘
      │          │              │                 │
      ▼          ▼              ▼                 ▼
┌──────────┐ ┌────────────┐ ┌──────────────┐ ┌────────────────┐
│Maintenance│ │  Object    │ │  QR Follow   │ │  Navigation    │
│ Monitor   │ │ Detection  │ │     Node     │ │    Manager     │
└─────┬─────┘ └─────┬──────┘ └──────┬───────┘ └────────────────┘
      │             │                │
      │             ▼                │
      │    turtlebot3/perception/*   │
      │                              │
      ▼                              │
turtlebot3/alerts                    │
                                     │
                                     ▼
                              Robot Movement
```

## Project Structure

```
turtlebot3_automation/
├── Subsystems
│   ├── maintenance/          # Health monitoring
│   ├── perception/           # Object detection (YOLO)
│   ├── navigation/           # Autonomous navigation (Nav2)
│   └── custom_features/      # QR code following
│
├── Support Systems
│   ├── utils/                # Logging, paths
│   └── setup_automation/     # Installation helpers
│
└── Configuration
    ├── config/               # YAML parameters
    ├── launch/               # Launch files
    └── tests/                # Unit tests
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## Acknowledgments

- Built on ROS 2 Foxy and Nav2 stack
- Uses YOLOv8 from Ultralytics
- Designed for ROBOTIS TurtleBot3 platform

## Contact

For questions or support, please open an issue on GitHub.
