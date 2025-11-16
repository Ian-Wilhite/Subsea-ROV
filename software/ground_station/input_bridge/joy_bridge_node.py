"""ROS 2 joystick bridge node placeholder."""

'''
This should be compatible with the 
 1) logitech F310 controller 
 2) Thrustmaster T-Flight Hotas X Flight Stick

'''

from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class JoyBridgeNode(Node):
    """ROS 2 node that ingests sensor_msgs/Joy data from a USB controller."""

    def __init__(self) -> None:
        super().__init__('joy_bridge')
        self._joy_topic: str = self.declare_parameter('joy_topic', '/joy').value
        self._cmd_vel_topic: str = self.declare_parameter('cmd_vel_topic', '/gcs/cmd_vel').value
        self._last_joy_msg: Optional[Joy] = None
        self._joy_subscription = self.create_subscription(
            Joy,
            self._joy_topic,
            self._joy_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            self._cmd_vel_topic,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self._axis_linear_x = int(self.declare_parameter('axis_linear_x', 1).value)
        self._axis_linear_y = int(self.declare_parameter('axis_linear_y', 0).value)
        self._axis_linear_z = int(self.declare_parameter('axis_linear_z', 4).value)
        self._axis_angular_z = int(self.declare_parameter('axis_angular_z', 3).value)
        self._button_z_up = int(self.declare_parameter('button_z_up', 4).value)
        self._button_z_down = int(self.declare_parameter('button_z_down', 5).value)
        self._scale_linear_x = float(self.declare_parameter('scale_linear_x', 1.0).value)
        self._scale_linear_y = float(self.declare_parameter('scale_linear_y', 1.0).value)
        self._scale_linear_z = float(self.declare_parameter('scale_linear_z', 1.0).value)
        self._scale_angular_z = float(self.declare_parameter('scale_angular_z', 1.0).value)
        self._button_z_scale = float(self.declare_parameter('button_z_scale', 1.0).value)
        self._deadband_linear_x = float(self.declare_parameter('deadband_linear_x', 0.0).value)
        self._deadband_linear_y = float(self.declare_parameter('deadband_linear_y', 0.0).value)
        self._deadband_linear_z = float(self.declare_parameter('deadband_linear_z', 0.0).value)
        self._deadband_angular_z = float(self.declare_parameter('deadband_angular_z', 0.0).value)
        self._button_arm = int(self.declare_parameter('button_arm', 7).value)
        self._button_disarm = int(self.declare_parameter('button_disarm', 6).value)
        self._button_camera_front = int(self.declare_parameter('button_camera_front', 2).value)
        self._button_camera_bottom = int(self.declare_parameter('button_camera_bottom', 3).value)
        self._button_camera_toggle = int(self.declare_parameter('button_camera_toggle', 1).value)
        self._arm_topic: str = self.declare_parameter('arm_topic', '/gcs/arm').value
        self._camera_topic: str = self.declare_parameter('camera_topic', '/gcs/camera_select').value
        self._telemetry_diagnostics_topic: str = self.declare_parameter(
            'telemetry_diagnostics_topic',
            '/gcs/telemetry_bridge/diagnostics',
        ).value
        self._telemetry_diagnostics_period = float(
            self.declare_parameter('telemetry_diagnostics_period', 1.0).value,
        )
        self._arm_publisher = self.create_publisher(
            Bool,
            self._arm_topic,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self._camera_publisher = self.create_publisher(
            String,
            self._camera_topic,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self._armed_state: bool = False
        self._active_camera: str = 'front'
        self._last_button_states: List[int] = []
        self._telemetry_diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            self._telemetry_diagnostics_topic,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        diagnostics_period = 0.1 if self._telemetry_diagnostics_period <= 0.0 else self._telemetry_diagnostics_period
        self._telemetry_diagnostics_timer = self.create_timer(
            diagnostics_period,
            self._publish_telemetry_heartbeat,
        )
        self.get_logger().info(f'Listening for Joy messages on {self._joy_topic}')
        self.get_logger().info(f'Publishing Twist messages on {self._cmd_vel_topic}')

    def _joy_callback(self, joy_msg: Joy) -> None:
        """Cache the latest Joy message for downstream command generation."""
        self._last_joy_msg = joy_msg
        cmd_vel = self._map_joy_to_twist(joy_msg)
        self._cmd_vel_publisher.publish(cmd_vel)
        self._publish_discrete_commands(joy_msg)
        self._last_button_states = list(joy_msg.buttons)

    def _map_joy_to_twist(self, joy_msg: Joy) -> Twist:
        twist = Twist()
        axes = joy_msg.axes
        buttons = joy_msg.buttons

        def axis_value(index: int) -> float:
            if 0 <= index < len(axes):
                return axes[index]
            return 0.0

        def button_value(index: int) -> int:
            if 0 <= index < len(buttons):
                return buttons[index]
            return 0

        def scaled_axis(index: int, scale: float, deadband: float) -> float:
            value = axis_value(index)
            return self._apply_deadband(value, deadband) * scale

        twist.linear.x = scaled_axis(self._axis_linear_x, self._scale_linear_x, self._deadband_linear_x)
        twist.linear.y = scaled_axis(self._axis_linear_y, self._scale_linear_y, self._deadband_linear_y)
        z_axis = scaled_axis(self._axis_linear_z, self._scale_linear_z, self._deadband_linear_z)
        z_buttons = (button_value(self._button_z_up) - button_value(self._button_z_down)) * self._button_z_scale
        twist.linear.z = z_axis + z_buttons
        twist.angular.z = scaled_axis(self._axis_angular_z, self._scale_angular_z, self._deadband_angular_z)
        return twist

    @staticmethod
    def _apply_deadband(value: float, deadband: float) -> float:
        threshold = abs(deadband)
        if threshold <= 0.0:
            return value
        if threshold >= 1.0 or abs(value) <= threshold:
            return 0.0
        magnitude = (abs(value) - threshold) / (1.0 - threshold)
        return magnitude if value > 0 else -magnitude

    def _button_rising_edge(self, index: int, buttons: List[int]) -> bool:
        if index < 0:
            return False
        current = 1 if 0 <= index < len(buttons) and buttons[index] else 0
        previous = 1 if 0 <= index < len(self._last_button_states) and self._last_button_states[index] else 0
        return current == 1 and previous == 0

    def _publish_discrete_commands(self, joy_msg: Joy) -> None:
        buttons = joy_msg.buttons
        if self._button_rising_edge(self._button_arm, buttons):
            self._armed_state = True
            self._arm_publisher.publish(Bool(data=True))
        elif self._button_rising_edge(self._button_disarm, buttons):
            self._armed_state = False
            self._arm_publisher.publish(Bool(data=False))

        camera_command: Optional[str] = None
        if self._button_rising_edge(self._button_camera_front, buttons):
            camera_command = 'front'
        elif self._button_rising_edge(self._button_camera_bottom, buttons):
            camera_command = 'bottom'
        elif self._button_rising_edge(self._button_camera_toggle, buttons):
            camera_command = 'bottom' if self._active_camera == 'front' else 'front'

        if camera_command:
            self._active_camera = camera_command
            self._camera_publisher.publish(String(data=camera_command))

    def _publish_telemetry_heartbeat(self) -> None:
        heartbeat = DiagnosticArray()
        heartbeat.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'telemetry_bridge'
        status.hardware_id = 'gcs_joy_bridge'
        status.level = DiagnosticStatus.OK
        status.message = 'Joy bridge heartbeat'
        last_joy_age = 'unavailable'
        if self._last_joy_msg is not None:
            last_stamp = Time.from_msg(self._last_joy_msg.header.stamp)
            age_seconds = (self.get_clock().now() - last_stamp).nanoseconds / 1e9
            if age_seconds < 0.0:
                age_seconds = 0.0
            last_joy_age = f'{age_seconds:.3f}'

        status.values = [
            KeyValue(key='armed', value='true' if self._armed_state else 'false'),
            KeyValue(key='active_camera', value=self._active_camera),
            KeyValue(key='last_joy_age_sec', value=last_joy_age),
        ]
        heartbeat.status.append(status)
        self._telemetry_diagnostics_publisher.publish(heartbeat)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
