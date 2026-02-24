"""
A module for detector node.
"""

from ultralytics import YOLO
from cv_bridge import CvBridge
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class DetectorNode(Node):
    """
    A detector node which can detect objects and aruco markers.
    """

    def __init__(self):
        super().__init__('detector_node')

        # Parameters
        self.declare_parameter('detection_mode', 'yolo')
        self.declare_parameter('aruco_dict_type', cv2.aruco.DICT_4X4_50)
        self.declare_parameter('yolo_model_path', 'weights/yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.4)

        # Pubs / Subs
        self.detections_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.detection_image_pub = self.create_publisher(Image, 'detection_image', 10)
        #------------------------------------------------------------------------------
        self.cam_info_sub = self.create_subscription(CameraInfo, 'camera_info', self.cam_info_callback, 10)
        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)

        # Others
        self.cv_bridge = CvBridge()
        self.marker_length = 0.05  # 5cm
        self.cam_matrix = None
        self.dist_coeffs = None

        # Check detector mode and load the corresponding detector
        if self.get_parameter('detection_mode').value == 'yolo':
            # Load YOLO model from path
            self.model = YOLO(self.get_parameter('yolo_model_path').value)
        elif self.get_parameter('detection_mode').value == 'aruco':
            # Initialize aruco detector
            self.dict_type = cv2.aruco.getPredefinedDictionary(self.get_parameter('aruco_dict_type').value)
            self.detector_parameters = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.dict_type, self.detector_parameters)
        else:
            pass

    
    def cam_info_callback(self, msg):
        """
        A callback that saves camera matrix and distortion coefficients.
        """

        # Convert cam matrix to 3x3 array
        self.cam_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    
    def image_callback(self, msg):
        """
        Main callback that processes images.

        Assuming incommimg message is Detection2DArray
        """

        detections = list()
        det_2d_array_msg = Detection2DArray()

        # Convert ros image to numpy array
        img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

        # Call detectors based on detection mode
        if self.get_parameter('detection_mode').value == 'aruco':
            detections = self.detect_aruco(img)
        elif self.get_parameter('detection_mode').value == 'yolo':
            detections = self.detect_yolo(img)
        else:
            pass

        # Iterate trough detections
        for detection in detections:
            det_2d_msg = Detection2D()
            ohwp = ObjectHypothesisWithPose()

            # Fill out the fields of detection message
            det_2d_msg.bbox.center.position.x = detection['bbox'][0]
            det_2d_msg.bbox.center.position.y = detection['bbox'][1]
            det_2d_msg.bbox.size_x = detection['bbox'][2]
            det_2d_msg.bbox.size_y = detection['bbox'][3]

            ohwp.hypothesis.class_id = detection['class_id']
            ohwp.hypothesis.score = detection['score']

            det_2d_msg.results.append(ohwp)

            det_2d_array_msg.detections.append(det_2d_msg)
        
        # Fill out the header then publish
        det_2d_array_msg.header = msg.header
        self.detections_pub.publish(det_2d_array_msg)

        # Draw bbox then publish the image
        for detection in detections:
            bbox = detection['bbox']

            # Draw bbox
            cv2.rectangle(
                img=img, 
                pt1=(int(bbox[0] - bbox[2] / 2), int(bbox[1] - bbox[3] / 2)),     # (x - w/2, y - h/2)
                pt2=(int(bbox[0] + bbox[2] / 2), int(bbox[1] + bbox[3] / 2)),     # (x + w/2, y + h/2)
                color=(255, 0, 0),                                             # BGR format
                thickness=3
            )

            # Put label
            cv2.putText(
                img=img,
                text=detection['class_id'],
                org=(int(bbox[0] - bbox[2] / 2), int(bbox[1] - bbox[3] / 2 - 10)),
                color=(0, 255, 0),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.6,
                thickness=3
            )

        # Convert image back to ros image and publish ros image
        ros_img = self.cv_bridge.cv2_to_imgmsg(img)
        self.detection_image_pub.publish(ros_img)

    
    def detect_aruco(self, img):
        """
        A function that detects aruco markers.
        """

        # Convert image to grayscale for reducing channel size
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        detections = []
        
        # Check if ids exists
        if ids is None:
            return detections
        
        # Iterate trough markers
        for corner, marker_id in zip(corners, ids):
            # Define image points (solvePnp expects shape of (4, 2))
            img_points = corner.reshape(4, 2)
            
            # Calculate bounding box
            xs = img_points[:, 0]
            ys = img_points[:, 1]
            center_x = float(np.mean(xs))
            center_y = float(np.mean(ys))
            width = float(np.max(xs) - np.min(xs))
            height = float(np.max(ys) - np.min(ys))
            
            detection = {
                'bbox': (center_x, center_y, width, height),
                'class_id': f'aruco_{marker_id[0]}',
                'score': 1.0,
            }
            
            # Pose estimation (if there is camera matrix)
            if self.cam_matrix is not None and self.dist_coeffs is not None:
                obj_points = np.array([
                    [-self.marker_length/2,  self.marker_length/2, 0],
                    [ self.marker_length/2,  self.marker_length/2, 0],
                    [ self.marker_length/2, -self.marker_length/2, 0],
                    [-self.marker_length/2, -self.marker_length/2, 0],
                ], dtype=np.float32)
                
                # Apply pose estimation
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points.astype(np.float32),
                    self.cam_matrix, self.dist_coeffs
                )
                
                # Flatten the vectors
                if success:
                    detection['tvec'] = tvec.flatten()
                    detection['rvec'] = rvec.flatten()
            
            detections.append(detection)
        
        return detections
    

    def detect_yolo(self, img):
        """
        A function that detects objects using YOLO.
        """

        # Take out the results
        detections = []
        results = self.model(img)

        # Iterate trough results
        for result in results:
            for box in result.boxes:  # Every box is a detection
                x, y, w, h = box.xywh[0].tolist()  # tensor -> python float
                
                # Take the most confident label and confidence
                conf = float(box.conf[0])
                class_id = result.names[int(box.cls[0])]

                # Confidence threshold filter
                if conf < self.get_parameter('conf_threshold').value:
                    continue

                detection = {                  # Example:
                    'bbox': (x, y, w, h),      #    Bounding box
                    'class_id': class_id,      #    string: "bottle"
                    'score': conf,             #    float: 0.87
                }

                detections.append(detection)

        return detections
    

def main():
    """
    Function that starts and ends node lifecycle.
    """

    # Start
    rclpy.init()
    node = DetectorNode()
    rclpy.spin(node)

    # End
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()