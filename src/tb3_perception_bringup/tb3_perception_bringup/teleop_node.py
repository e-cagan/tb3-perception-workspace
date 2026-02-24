"""
Module for making teleop controls easy.

Controlled with WASD (forward, left, backward, right)
Q (to quit)
SPACE (to stop the robot)
"""

import termios
import select
import tty
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class TeleopNode(Node):
    """
    Teleop keyboard control class.

    Controlled with WASD (forward, left, backward, right)
    
    Q (to quit)
    
    SPACE (to stop the robot)
    """

    def __init__(self):
        super().__init__('teleop_node')

        # Publisher
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(timer_period_sec=0.05, callback=self.timer_callback)

        self.current_msg = Twist()
        self.old_settings = termios.tcgetattr(sys.stdin)
        self.linear_speed = 0.3
        self.angular_speed = 0.5

        # Usage guide banner.
        print("=== Teleop Keyboard Controller ===")
        print("W: Forward")
        print("S: Backward")
        print("A: Turn Left")
        print("D: Turn Right")
        print("SPACE: Stop")
        print("Q: Quit")
        print("==================================")

        # Set terminal to raw mode for listening keys
        tty.setraw(sys.stdin)


    def timer_callback(self):
        """
        Timer callback function for teleop node.
        """

        # Check if there is an input to read
        if select.select([sys.stdin], [], [], 0)[0]:
            # Read the key
            key = sys.stdin.read(1)

            # Check the key matches one of the desired inputs
            if key == 'w':
                self.current_msg.linear.x = self.linear_speed
                self.current_msg.angular.z = 0.0
                self.get_logger().info(f"Pressing {key}. Going forward!")
            elif key == 's':
                self.current_msg.linear.x = -self.linear_speed
                self.current_msg.angular.z = 0.0
                self.get_logger().info(f"Pressing {key}. Going backward!")
            elif key == 'a':
                self.current_msg.angular.z = self.angular_speed
                self.current_msg.linear.x = 0.0
                self.get_logger().info(f"Pressing {key}. Going left!")
            elif key == 'd':
                self.current_msg.angular.z = -self.angular_speed
                self.current_msg.linear.x = 0.0
                self.get_logger().info(f"Pressing {key}. Going right!")
            elif key == ' ':
                self.current_msg.linear.x = 0.0
                self.current_msg.angular.z = 0.0
                self.get_logger().info(f"Pressed space. Stopping!")
            elif key == 'q':
                self.get_logger().info(f"Pressed {key}. Quitting!")
                self.timer.cancel()
                raise SystemExit

        # Publish the message
        self.pub.publish(self.current_msg)
            

def main():
    try:
        rclpy.init()
        node = TeleopNode()
        rclpy.spin(node)
    finally:
        # Reload old terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()