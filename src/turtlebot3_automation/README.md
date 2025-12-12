# turtlebot3_automation ROS 2 Package

This ROS 2 package provides automation tooling for TurtleBot3 workflows on ROS 2 Humble, covering:

* Environment setup automation assisted by official documentation references.
* Maintenance automation with periodic diagnostics and alerting hooks.
* Autonomous navigation orchestration with Nav2 for SLAM or map-based missions.
* Object detection integration leveraging YOLOv8 and publishing to ROS 2 topics with RViz markers.
* A custom QR-code following feature showcasing extensibility.

Install the package alongside TurtleBot3 dependencies within a `colcon` workspace and use the provided console entrypoints or launch files to bring up individual automation capabilities.
