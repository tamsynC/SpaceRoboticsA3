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

#Including imports just in case

class NodeExplore:
    def __init__(self):
        self.Visited = []
        self.Unvisisted = []
        self.AllNodes = []
        
    def NodeToPose(self, node):
        pose_2d = Pose2D()
        pose_2d.x = node.x
        pose_2d.y = node.y
        pose_2d.theta = node.theta
        
        return pose_2d

class Node:
    #Why a node and not a point? Might need to add other info to nodes later
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0