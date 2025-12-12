"""Maintenance monitoring node that aggregates TurtleBot3 health metrics."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from ..infrastructure.logging import configure_logging

_ALERT_TOPIC = "turtlebot3/alerts"


class MaintenanceMonitor(Node):
    """Collect battery, diagnostics, and motion telemetry with alerting."""

    def __init__(self) -> None:
        super().__init__("maintenance_monitor")

        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("battery_critical_voltage", 10.8)
        self.declare_parameter("battery_warning_voltage", 11.4)
        self.declare_parameter("report_period", 30.0)
        self.declare_parameter("log_dir", str(Path.home() / ".ros" / "turtlebot3_automation"))

        self._battery_voltage: Optional[float] = None
        self._battery_percentage: Optional[float] = None
        self._last_motion: Optional[Twist] = None
        self._diagnostics: Dict[str, int] = {}
        self._last_diag_timestamp: Optional[TimeMsg] = None

        log_dir = Path(self.get_parameter("log_dir").get_parameter_value().string_value)
        log_dir.mkdir(parents=True, exist_ok=True)
        self._audit_log = log_dir / "maintenance_report.jsonl"
        # Use console=False to avoid double printing (ROS logger handles console)
        self._file_logger = configure_logging(self.get_name(), log_dir / "maintenance.log", console=False)

        self._alert_pub = self.create_publisher(String, _ALERT_TOPIC, 10)
        self._battery_sub = self.create_subscription(
            BatteryState,
            self.get_parameter("battery_topic").get_parameter_value().string_value,
            self._on_battery,
            10,
        )
        self._diagnostic_sub = self.create_subscription(
            DiagnosticArray,
            self.get_parameter("diagnostics_topic").get_parameter_value().string_value,
            self._on_diagnostics,
            10,
        )
        self._motion_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            self._on_motion,
            10,
        )

        period = self.get_parameter("report_period").get_parameter_value().double_value
        self._report_timer = self.create_timer(period, self._report)

        self.get_logger().info(f"Maintenance monitor started with report period {period:.1f} s")

    def _emit_alert(self, message: str) -> None:
        msg = String()
        msg.data = message
        self._alert_pub.publish(msg)
        self.get_logger().warn(message)
        self._file_logger.warning(message)

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_voltage = msg.voltage
        self._battery_percentage = msg.percentage
        self.get_logger().debug(f"Battery update: {msg.voltage:.2f} V | {msg.percentage * 100.0:.1f}%")

        crit = self.get_parameter("battery_critical_voltage").get_parameter_value().double_value
        warn = self.get_parameter("battery_warning_voltage").get_parameter_value().double_value
        if msg.voltage <= crit:
            self._emit_alert(f"Battery critical: {msg.voltage:.2f} V")
        elif msg.voltage <= warn:
            self.get_logger().warn(f"Battery low: {msg.voltage:.2f} V")

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            self._diagnostics[status.name] = status.level
            if status.level >= 2:
                self._emit_alert(f"Diagnostic ERROR from {status.name}: {status.message}")
            elif status.level == 1:
                self.get_logger().warn(f"Diagnostic WARN from {status.name}: {status.message}")
        self._last_diag_timestamp = msg.header.stamp

    def _on_motion(self, msg: Twist) -> None:
        self._last_motion = msg

    def _report(self) -> None:
        diagnostic_summary = {
            name: level
            for name, level in sorted(self._diagnostics.items())
            if level > 0
        }

        data = {
            "timestamp": time.time(),
            "battery_voltage": self._battery_voltage,
            "battery_percentage": self._battery_percentage,
            "diagnostics": diagnostic_summary,
            "last_diagnostic_time": self._time_to_float(self._last_diag_timestamp),
            "cmd_vel": self._twist_to_dict(self._last_motion),
        }

        report_msg = f"Report | V={data['battery_voltage'] or -1.0:.2f} | pct={(data['battery_percentage'] or 0.0) * 100.0:.1f} | diag={diagnostic_summary}"
        self.get_logger().info(report_msg)
        self._file_logger.info(report_msg)

        with self._audit_log.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(data) + "\n")

    @staticmethod
    def _time_to_float(msg: Optional[TimeMsg]) -> Optional[float]:
        if msg is None:
            return None
        return msg.sec + msg.nanosec * 1e-9

    @staticmethod
    def _twist_to_dict(msg: Optional[Twist]) -> Optional[Dict[str, float]]:
        if msg is None:
            return None
        return {
            "linear_x": msg.linear.x,
            "linear_y": msg.linear.y,
            "linear_z": msg.linear.z,
            "angular_x": msg.angular.x,
            "angular_y": msg.angular.y,
            "angular_z": msg.angular.z,
        }


def main() -> None:
    rclpy.init()
    node = MaintenanceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - manual shutdown
        node.get_logger().info("Maintenance monitor interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
