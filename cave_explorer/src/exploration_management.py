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
        
        self.thresholdRange = 3 #adjust as needed
        self.exclusivityRadius = 4 #adjust as needed
        
    def NodeToPose(self, node):
        pose_2d = Pose2D()
        pose_2d.x = node.x
        pose_2d.y = node.y
        pose_2d.theta = node.theta
        
        return pose_2d
    
    #Condense this for efficiency later
    def CreateNodes(self, laserdata, odom):
        #Take in laser data between goals -> generate new nodes that arent within "exclusivity radius" of another node and add them to the unvisited set
        
        angleCount = 0
        for range in laserdata.ranges:
            #infinite is good -> means no collisions
            #print(range)
            if range == np.inf:
                pass
                #range = 5 #too big and we get unintended goals, too small and we just shouldn't consider them
                #
                #currTheta = laserdata.angle_min + (laserdata.angle_increment * angleCount)
                #currX = range * math.cos(currTheta) + odom.x
                #currY = range * math.sin(currTheta) + odom.y
                #
                #currPoint = Node(currX, currY, currTheta)
                #
                #if not self.isOccupied(currPoint):
                #    self.AllNodes.append(currPoint)
                #    self.Unvisisted.append(currPoint)
                #    print("Made node at: ", currPoint.x, currPoint.y)
            
            #threshold range is the minimum range to create a node to, prevents nodes that don't make meaningful exploration progress
            elif range < self.thresholdRange:
                
                range = range -0.5 #keep it from crashing
                
                currTheta = laserdata.angle_min + (laserdata.angle_increment * angleCount)
                currX = range * math.cos(currTheta) + odom.x
                currY = range * math.sin(currTheta) + odom.y
                
                currPoint = Node(currX, currY, currTheta)
                
                if not self.isOccupied(currPoint):
                    self.AllNodes.append(currPoint)
                    self.Unvisisted.append(currPoint)
                    print("Made node at: ", currPoint.x, currPoint.y)
            
            angleCount += 1       

    def isOccupied(self, Point):
        #Starts false, if a node is detected in range flag true that the space is "occupied"
        #Notably using absolute distances, does not account for Line of Sight, adjust exclusivity radius as needed for best results.
        for node in self.AllNodes:
            distBetween = math.sqrt(pow(node.x - Point.x, 2) + pow(node.y - Point.y, 2))
            #print("Dist between: ", distBetween)
            if distBetween < self.exclusivityRadius:
                return True
        return False    

class Node:
    

    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta