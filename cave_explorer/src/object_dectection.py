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

#Depth image
from sensor_msgs.msg import Image

class ObjectDetector:
    def __init__(self):
        self.model = YOLO('/home/cajwill/catkin_ws/src/SpaceRoboticsA3/cave_explorer/src/YOLOv2.pt')  # Load YOLOv8 model on GPU
        self.bridge = CvBridge()

        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback, queue_size=10)
        self.depth_image = None

    def process_image(self, image_msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Reduce image resolution for faster processing (optional)
            cv_image = cv2.resize(cv_image, (640, 480))

            # Run YOLO detection on the image
            results = self.model(cv_image)

            objectDepthAngles = []

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
                    if conf > 0.5:
                        text = f"{label}: {conf:.2f}"
                        cv2.putText(cv_image, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 32, 240), 2)

                    #avg depth
                    avg_depth = self.calculate_average_depth(x1, y1, x2, y2)
                    angle = self.calculate_angle(x1, y1, x2, y2)
                    objtype = class_id

                    objTuple = (avg_depth, angle, objtype)

                    objectDepthAngles.append(objTuple)

                    rospy.loginfo(f"Object {label} is at an average depth of {avg_depth:.2f} meters and angle {angle:.2f} degrees.")

            # Convert the processed OpenCV image back to a ROS Image message
            output_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

            return output_image_msg, objectDepthAngles

        except CvBridgeError as e:
            rospy.logerr(f"CvBridgeError: {e}")


    def depth_callback(self, depth_msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")  # Depth image in meters

        except CvBridgeError as e:
            rospy.logerr(f"CvBridgeError: {e}")


    def what_is_this(self, results):

        thing = []

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                label = self.model.names[class_id]

                # x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                # mid_point = (x1 + x2) / 2               

                # tuple = (mid_point, label)

                thing.append(label)

        return thing


    def calculate_average_depth(self, x1, y1, x2, y2):
        if self.depth_image is None:
            return float('nan') #no image avaliable
        
        depth_roi = self.depth_image[y1:y2, x1:x2]

        valid_depths = depth_roi[depth_roi > 0]

        if len(valid_depths) > 0:
            avg_depth = np.mean(valid_depths)

        else:
            avg_depth = float('nan')

        print("avg_depth: ", avg_depth)

        return avg_depth

    def calculate_angle(self, x1, y1, x2, y2):
        mid_point = (x1 + x2) / 2

        horizontal_fov = 60.0 #degrees for the camera -> need to find specs

        angle_per_pixel = horizontal_fov / 640

        angle = (mid_point - (640 / 2)) * angle_per_pixel

        print("angle: ", angle)

        return angle


