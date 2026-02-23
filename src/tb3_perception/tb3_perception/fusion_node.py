"""
Module for unifying detection data with LIDAR data.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan, CameraInfo
from geometry_msgs.msg import PoseStamped


# Helpers
def is_valid(idx):
    """
    a helper function that filters out 0.0 and inf values on laser scan which are invalid values.
    """

    if idx != 0.0 and idx != math.inf:
        return idx


class FusionNode(Node):
    """
    A node that unifies detection data with LIDAR data.
    """

    def __init__(self):
        super().__init__('fusion_node')

        # Pubs/Subs
        # ----------------------------------------------------------------------------------------------------
        self.det_sub = self.create_subscription(Detection2DArray, 'detections', self.detection_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, 'camera/camera_info', self.cam_info_callback, 10)
        # ----------------------------------------------------------------------------------------------------
        self.target_pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)

        # Other vars
        self.last_scan = None
        self.cam_matrix = None
        self.dist_coeffs = None

    
    def cam_info_callback(self, msg):
        """
        A callback function which sets up the camera matrix.
        """

        # Convert cam matrix to 3x3 array
        self.cam_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)


    def scan_callback(self, msg):
        """
        A callback function which sets up the last scan LaserScan message.
        """

        # Set the message
        self.last_scan = msg

    
    def detection_callback(self, msg):
        """
        A callback which unifies detection and LIDAR data.
        """

        # Create target publish message
        msg_pub = PoseStamped()

        # Check a scan and camera matrix exists
        if self.last_scan is None:
            return
        if self.cam_matrix is None:
            return
        if self.dist_coeffs is None:
            return
        
        # Take the fx and cx
        fx = self.cam_matrix[0][0]
        cx = self.cam_matrix[0][2]

        # Vars to decide closest detection
        x = 0.0
        y = 0.0
        best_distance = None
        best_angle = None

        # Iterate trough detections to take centers of the bbox
        for detection in msg.detections:
            # Take the center x pixel
            u = detection.bbox.center.position.x

            # Convert pixels to angles
            angle = np.arctan2(u - cx, fx)
            angular_width = np.arctan2(detection.bbox.size_x / 2, fx)
            
            # Turn negative angle to positive by adding 2 * pi (radian) if negative
            if angle < 0:
                angle = 2 * math.pi + angle
            
            # Calculate indexes
            index = angle / self.last_scan.angle_increment
            if index < 0:
                index = len(self.last_scan.ranges) + index

            # Interval slicing for angle and index
            min_angle = angle - angular_width
            max_angle = angle + angular_width

            min_index = int(min_angle / self.last_scan.angle_increment)
            max_index = int(max_angle / self.last_scan.angle_increment)

            # Turn negative indexes to positive
            if min_index < 0:
                min_index = len(self.last_scan.ranges) + min_index
            if max_index < 0:
                max_index = len(self.last_scan.ranges) + max_index

            # Take ranged indexes and filter them. Lastly take the minimum of it
            distances = self.last_scan.ranges[min_index:max_index]
            distances = list(filter(is_valid, distances))
            distance = min(distances) if distances else None

            # Update the best distance and angle
            if distance is not None and (best_distance is None or distance < best_distance):
                best_distance = distance
                best_angle = angle

        # Check if there is a best distance
        if best_distance is not None:
            # Take x, y by converting polar to cartesian
            x = best_distance * np.cos(best_angle)
            y = best_distance * np.sin(best_angle)

            # Fill out the necessary areas of PoseStamped message
            msg_pub.header.frame_id = 'base_link'
            msg_pub.pose.position.x = x
            msg_pub.pose.position.y = y

            # Publish the message
            self.target_pose_pub.publish(msg_pub)
        else:
            pass


def main():
    """
    A function that starts and ends node lifecycle.
    """

    # Lifecycle
    rclpy.init()
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()