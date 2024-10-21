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

#Currently: needs to not just explore furthest node, but instead search "new" areas

class NodeExplore:
    def __init__(self):
        self.Visited = []
        self.Unvisisted = []
        self.AllNodes = []
        
        self.thresholdRange = 6 #min distance for laserscan to care
        self.exclusivityRadius = 8 #prevent other nodes from being within this range
        
        
        
    def NodeToPose(self, node):
        pose_2d = Pose2D()
        pose_2d.x = node.x
        pose_2d.y = node.y
        pose_2d.theta = node.theta
        
        return pose_2d
    
    #Condense this for efficiency later
    def CreateNodes(self, laserdata, odom, isFirstScan):
        #Take in laser data between goals -> generate new nodes that arent within "exclusivity radius" of another node and add them to the unvisited set
        #print("Odom values: ", odom.x, odom.y)
        
        if isFirstScan:
            InitialPoint = Node(odom.x, odom.y, odom.theta)
            self.AllNodes.append(InitialPoint)
        
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
            elif range > self.thresholdRange:
                
                range = range - 3 #keep it from crashing
                
                currTheta = laserdata.angle_min + (laserdata.angle_increment * angleCount) + odom.theta
                wrappedTheta = currTheta
                outOfRange = False
                Over = True
                
                if wrappedTheta > 2 * math.pi:
                    outOfRange = True
                elif  wrappedTheta < -2 * math.pi:
                    outOfRange = True
                    Over = False
                
                while outOfRange:
                    
                    if Over == True:
                        wrappedTheta -= 2 * math.pi
                    else:
                        wrappedTheta += 2 * math.pi
                    
                    if -2 * math.pi <= wrappedTheta and wrappedTheta <= 2 * math.pi:
                        outOfRange = False
                
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