#!/usr/bin/env python3
"""
Joystick teleop node – converts sensor_msgs/Joy (from arduino_joy)
into geometry_msgs/Twist for differential-drive control.

Publishes on 'cmd_vel_joy' so twist_mux can multiplex it.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        # Configurable parameters
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('deadzone', 0.1)

        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        
        self.was_zero = False

        self.pub = self.create_publisher(Twist, 'cmd_vel_joy', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info(
            f'Joy teleop started – linear={self.max_linear} m/s, '
            f'angular={self.max_angular} rad/s, deadzone={self.deadzone}'
        )

    def joy_callback(self, msg: Joy):
        twist = Twist()

        if len(msg.axes) >= 2:
            # axes[1] = Y stick → forward/backward
            # axes[0] = X stick → left/right rotation
            raw_linear = msg.axes[1]
            raw_angular = msg.axes[0]

            # Apply deadzone
            if abs(raw_linear) < self.deadzone:
                raw_linear = 0.0
            if abs(raw_angular) < self.deadzone:
                raw_angular = 0.0

            # Eğer joystick tamamen ortadaysa (sıfır), sürekli 0 yayınlama.
            # Sadece ilk bırakıldığında 1 kez 0 yayınla ki motorlar dursun.
            # Yoksa otonom sürüşü (cmd_vel_nav) sonsuza kadar bloke eder!
            if raw_linear == 0.0 and raw_angular == 0.0:
                if not self.was_zero:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.pub.publish(twist)
                    self.was_zero = True
                return

            self.was_zero = False
            twist.linear.x = raw_linear * self.max_linear
            twist.angular.z = raw_angular * self.max_angular

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
