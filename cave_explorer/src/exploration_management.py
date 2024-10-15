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

#Including imports just in case

class NodeExplore:
    def __init__(self):
        self.Visited = []
        self.Unvisisted = []
        self.AllNodes = []
        
        self.thresholdRange = 5 #adjust as needed
        self.exclusivityRadius = 5 #adjust as needed
        
    def NodeToPose(self, node):
        pose_2d = Pose2D()
        pose_2d.x = node.x
        pose_2d.y = node.y
        pose_2d.theta = node.theta
        
        return pose_2d
    
    #Condense this for efficiency later
    def CreateNodes(self, laserdata):
        #Take in laser data between goals -> generate new nodes that arent within "exclusivity radius" of another node and add them to the unvisited set
        
        angleCount = 0
        for range in laserdata.ranges:
            #infinite is good -> means no collisions
            if range == np.inf:
                range = laserdata.range_max - 5
                
                currTheta = laserdata.angle_min + (laserdata.angle_increment * angleCount)
                currX = range * math.cos(currTheta)
                currY = range * math.sin(currTheta)
                
                currPoint = Node(currX, currY, currTheta)
                
                if not self.isOccupied(currPoint):
                    self.AllNodes.append(currPoint)
                    self.Unvisisted.append(currPoint)
            
            #threshold range is the minimum range to create a node to, prevents nodes that don't make meaningful exploration progress
            elif range < self.thresholdRange:
                
                currTheta = laserdata.angle_min + (laserdata.angle_increment * angleCount)
                currX = range * math.cos(currTheta)
                currY = range * math.sin(currTheta)
                
                currPoint = Node(currX, currY, currTheta)
                
                if not self.isOccupied(currPoint):
                    self.AllNodes.append(currPoint)
                    self.Unvisisted.append(currPoint)
            
            angleCount += 1       

    def isOccupied(self, Point):
        #Starts false, if a node is detected in range flag true that the space is "occupied"
        #Notably using absolute distances, does not account for Line of Sight, adjust exclusivity radius as needed for best results.
        withinRange = False
        for node in self.AllNodes:
            distBetween = math.sqrt(pow(node.x - Point.x, 2) + pow(node.y - Point.y, 2))
            if distBetween < self.exclusivityRadius:
                withinRange = True
        return withinRange     

class Node:
    
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
    
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta