# Changelog

All notable changes to the TurtleBot3 Automation Suite will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Complete project structure improvements
- Makefile for common development tasks
- Docker support for containerized deployment
- CI/CD pipeline with GitHub Actions
- Pre-commit hooks for code quality
- Comprehensive documentation and contributing guidelines
- MIT License

### Changed
- Updated README with complete setup instructions
- Improved environment management (ros-humble conda environment)
- Enhanced troubleshooting section
- Migrated from ROS Foxy to ROS Humble (Ubuntu 22.04)
- Updated Python requirement from 3.8 to 3.10

### Fixed
- Environment setup issues resolved
- Better handling of ROS Humble dependencies

## [1.0.0] - 2024-12-07

### Added
- **Navigation Automation**: Complete SLAM and autonomous navigation using Nav2 stack
- **Object Detection**: Real-time YOLOv8 inference with ROS 2 message publishing
- **Maintenance Monitoring**: System health monitoring with battery and diagnostics
- **QR Code Following**: Custom QR detection and following behavior
- **Modular Architecture**: ROS 2 nodes with unified CLI and launch interfaces
- **Comprehensive Configuration**: YAML-based parameter management
- **Testing Framework**: Parameter validation and smoke tests

### Technical Features
- ROS 2 Humble compatibility
- Python 3.10 support
- Modular node architecture
- Launch file configurations
- Parameter validation
- Error handling and logging

### Documentation
- Complete setup instructions
- Usage examples
- Configuration guides
- Troubleshooting tips