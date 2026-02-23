"""
A module for approach node.
"""

import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import Buffer, TransformListener

import rclpy
from rclpy.time import Time
from rclpy.node import Node

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped


class ApproachNode(Node):
    """
    A node that approaches the detected target.
    """

    def __init__(self):
        super().__init__('approach_node')

        # Parameters
        self.declare_parameter('stop_distance', 0.3)

        # Subs
        self.target_sub = self.create_subscription(PoseStamped, 'target_pose', self.target_callback, 10)

        # Tf2 and navigator
        self.nav = BasicNavigator()
        self.buffer = Buffer()
        self.transform = TransformListener(self.buffer, self)

        # State variables
        self.stop_distance = self.get_parameter('stop_distance').value
        self.navigating = False

    
    def target_callback(self, msg):
        """
        Callback function that navigates to detection.
        """

        # Take x and y to calculate distance
        x = msg.pose.position.x
        y = msg.pose.position.y
        distance = np.sqrt(x**2 + y**2)

        # Check for stopping
        if distance < self.stop_distance:
            if self.navigating:
                # Cancel the task
                self.nav.cancelTask()
                self.get_logger().info(f"Close enough to goal. Distance: {distance}")
                return
            return
        
        # Convert base_link -> map with tf2
        try:
            transform = self.buffer.lookup_transform('map', 'base_link', Time())
        except Exception as e:
            self.get_logger().error(f"Exception occured: {e}")
            return

        # Retrieve Pose (not PoseStamped)
        pose = do_transform_pose(msg.pose, transform)

        # Create new pose stamped message
        new_msg = PoseStamped()
        new_msg.header.frame_id = 'map'
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.pose = pose

        # Go to pose
        self.nav.goToPose(new_msg)
        self.navigating = True


def main():
    """
    Main function for node lifecycle.
    """

    # Lifecycle
    rclpy.init()
    node = ApproachNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()