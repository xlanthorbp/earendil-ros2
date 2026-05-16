#!/usr/bin/env python3
"""
Tank-style keyboard teleop node for the Earendil Bot rover.
  W / S  →  forward / backward
  A / D  →  rotate left / rotate right
  Q      →  quit
"""

import sys
import tty
import termios
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


HELP_MSG = """
╔══════════════════════════════════╗
║     Tank Teleop Controller       ║
╠══════════════════════════════════╣
║   W  →  Forward                  ║
║   S  →  Backward                 ║
║   A  →  Rotate Left              ║
║   D  →  Rotate Right             ║
║   Q  →  Quit                     ║
╚══════════════════════════════════╝
"""


class TankTeleop(Node):
    def __init__(self):
        super().__init__('tank_teleop')

        self.declare_parameter('linear_speed', 2.0)
        self.declare_parameter('angular_speed', 1.5)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        self.pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.get_logger().info('Tank teleop started – press W/S/A/D to move, Q to quit')

    def publish_twist(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    def stop(self):
        self.publish_twist(0.0, 0.0)


def _key_available(timeout=0.1):
    """Return True if a key press is waiting on stdin."""
    return select.select([sys.stdin], [], [], timeout)[0] != []


def main(args=None):
    rclpy.init(args=args)
    node = TankTeleop()

    # Save terminal settings so we can restore on exit
    old_settings = termios.tcgetattr(sys.stdin)

    try:
        # Put terminal in raw mode (character-at-a-time, no echo)
        tty.setraw(sys.stdin.fileno())
        print(HELP_MSG)

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            if _key_available(0.05):
                key = sys.stdin.read(1).lower()

                if key == 'q' or key == '\x03':  # q or Ctrl-C
                    node.stop()
                    break
                elif key == 'w':
                    node.publish_twist(node.linear_speed, 0.0)
                elif key == 's':
                    node.publish_twist(-node.linear_speed, 0.0)
                elif key == 'a':
                    node.publish_twist(0.0, node.angular_speed)
                elif key == 'd':
                    node.publish_twist(0.0, -node.angular_speed)
                else:
                    node.stop()
            else:
                # No key pressed → stop the robot
                node.stop()

    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\nTank teleop stopped.')


if __name__ == '__main__':
    main()
