import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Pose
from cv_bridge import CvBridge


class SunTrackingNode(Node):
    def __init__(self):
        super().__init__('sun_tracking_node')
        self.get_logger().info('Sun Tracking Node has been started.')
    
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
    
        self.publisher = self.create_publisher(Pose, 'sun/pose', 10)
        self.bridge = CvBridge()

        self.detector = 

    def set_up_detector(self):
        # SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        params.filterByColor = False

        # Change thresholds
        params.minThreshold = 200
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 50

        # Filter by Circularity
        params.filterByCircularity = True
#         params.minCircularity = 0.75
        params.minCircularity = 0.5

        # Filter by Convexity
        params.filterByConvexity = True
#         params.minConvexity = 0.87
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # Create a detector with the parameters
        self.detector = cv2.SimpleBlobDetector_create(params)
        
        




def main(args=None):
    rclpy.init(args=args)
    sun_tracking_node = SunTrackingNode()
    rclpy.spin(sun_tracking_node)
    sun_tracking_node.destroy_node()
    rclpy.shutdown()

