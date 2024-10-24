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
        self.model = YOLO('/home/tamsyn2004/SpaceRoboticsA3/cave_explorer/src/YOLOv2.pt')  # Load YOLOv8 model on GPU
        self.bridge = CvBridge()
        # self.image_detections_pub_ = rospy.Publisher('detections_image', Image, queue_size=10)  # Increase queue size

    # def process_image(self, image_msg):
    #     try:
    #         # Convert the ROS Image message to an OpenCV image
    #         cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

    #         # Reduce image resolution
    #         cv_image = cv2.resize(cv_image, (640, 480))

    #         # Optionally, run YOLO detection here (currently not processing)
    #         results = self.model(cv_image)



    #         return output_image_msg

    #     except CvBridgeError as e:
    #         rospy.logerr(f"CvBridgeError: {e}")

    def process_image(self, image_msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Reduce image resolution for faster processing (optional)
            cv_image = cv2.resize(cv_image, (640, 480))

            # Run YOLO detection on the image
            results = self.model(cv_image)

            # Get the bounding box results and draw them on the image
            for result in results:
                # Loop through detections in each image
                for box in result.boxes:
                    # Extract bounding box coordinates, confidence, and class
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())  # Coordinates (top-left and bottom-right)
                    conf = box.conf[0]  # Confidence score
                    class_id = int(box.cls[0])  # Class index
                    label = self.model.names[class_id]  # Class label (name)

                    # print(".........", label, ".........") debuging 

                    # Draw the bounding box on the image r: 160, g: 32, b: 240

                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (160, 32, 240), 2)

                    # Draw the label and confidence score on the image
                    text = f"{label}: {conf:.2f}"
                    cv2.putText(cv_image, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 32, 240), 2)

            # Convert the processed OpenCV image back to a ROS Image message
            output_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

            return output_image_msg

        except CvBridgeError as e:
            rospy.logerr(f"CvBridgeError: {e}")


    def what_is_this(results):

        thing = []

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                label = self.model.names[class_id]

                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                mid_point = (x1 + x2) / 2               

                tuple = (mid_point, label)

                thing.append(tuple)

        return thing


