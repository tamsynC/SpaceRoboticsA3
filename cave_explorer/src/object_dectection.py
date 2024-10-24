import rospy
import roslib
import math
import cv2 # OpenCV2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import tf
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
import actionlib
import random
import copy
from threading import Lock
from enum import Enum
from sensor_msgs.msg import LaserScan
from ultralytics import YOLO  # Assuming you use YOLOv8 or later versions from ultralytics

class ObjectDetector:
    def __init__(self):
        self.model = YOLO('/home/cajwill/catkin_ws/src/SpaceRoboticsA3/cave_explorer/src/YOLOv2.pt')  # Load YOLOv8 model on GPU
        self.bridge = CvBridge()
        # self.image_detections_pub_ = rospy.Publisher('detections_image', Image, queue_size=10)  # Increase queue size

    def process_image(self, image_msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Reduce image resolution
            cv_image = cv2.resize(cv_image, (640, 480))

            # Optionally, run YOLO detection here (currently not processing)
            results = self.model(cv_image)
            annotated_image = results[0].plot()  # Use plot() to annotate image with bounding boxes

            # Convert the processed image back to ROS Image message
            output_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")

            # Publish the image to the new topic
            # self.image_detections_pub_.publish(output_image_msg)

            return output_image_msg, results

        except CvBridgeError as e:
            rospy.logerr(f"CvBridgeError: {e}")