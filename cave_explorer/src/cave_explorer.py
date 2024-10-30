#!/usr/bin/env python3
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
from exploration_management import Node, NodeExplore
from sensor_msgs.msg import LaserScan
from ultralytics import YOLO
from object_dectection import ObjectDetector
from itertools import permutations

from visualization_msgs.msg import Marker, MarkerArray
import visualization_msgs


def wrap_angle(angle):
    # Function to wrap an angle between 0 and 2*Pi
    while angle < 0.0:
        angle = angle + 2 * math.pi

    while angle > 2 * math.pi:
        angle = angle - 2 * math.pi

    return angle

def pose2d_to_pose(pose_2d):
    pose = Pose()

    pose.position.x = pose_2d.x
    pose.position.y = pose_2d.y

    pose.orientation.w = math.cos(pose_2d.theta / 2.0)
    pose.orientation.z = math.sin(pose_2d.theta / 2.0)

    return pose


class PlannerType(Enum):
    ERROR = 0
    MOVE_FORWARDS = 1
    RETURN_HOME = 2
    GO_TO_FIRST_ARTIFACT = 3
    RANDOM_WALK = 4
    RANDOM_GOAL = 5
    INTERSECTION_EXPLORE = 6
    GO_TO_ARTIFACT = 7
    TSP = 8
    # Add more!

class CaveExplorer:
    def __init__(self):

        # Variables/Flags for perception
        self.localised_ = False
        self.artifact_found_ = False

        # Variables/Flags for planning
        self.planner_type_ = PlannerType.ERROR
        self.reached_first_artifact_ = False
        self.returned_home_ = False
        self.goal_counter_ = 0 # gives each goal sent to move_base a unique ID

        # Initialise CvBridge
        self.cv_bridge_ = CvBridge()

        # Initialise NodeManagement
        self.nodes = NodeExplore()
        self.FirstScan = True
        self.currGoal = MoveBaseActionGoal()
        self.goingToArtifact = False
        self.artifactNodes = []
        self.artifactClasses = []
        self.artifactUnvisited = []
        self.artifactMaybe = []
        self.artifactMaybeCounts = []

        self.MarkerCount = 0

        #Initialise ObjectDetector
        self.detector = ObjectDetector()
        
        #Initialise Laserscan saving + subscriber
        self.laserSub = rospy.Subscriber('scan', LaserScan, self.LaserCallback, queue_size=10)
        self.laserData = LaserScan()

        #Advanced 4
        self.idealGoalOrder = self.TSP_Solver()
        self.patrolCounter = 1

        # Wait for the transform to become available
        rospy.loginfo("Waiting for transform from map to base_link")
        self.tf_listener_ = tf.TransformListener()

        while not rospy.is_shutdown() and not self.tf_listener_.canTransform("map", "base_link", rospy.Time(0.)):
            rospy.sleep(0.1)
            print("Waiting for transform... Have you launched a SLAM node?")        

        # Advertise "cmd_vel" publisher to control the robot manually -- though usually we will be controller via the following action client
        self.cmd_vel_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Action client for move_base
        self.move_base_action_client_ = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action...")
        self.move_base_action_client_.wait_for_server()
        rospy.loginfo("move_base connected")

        # Publisher for the camera detections
        self.image_detections_pub_ = rospy.Publisher('detections_image', Image, queue_size=1)

        # Read in computer vision model (simple starting point)
        self.computer_vision_model_filename_ = rospy.get_param("~computer_vision_model_filename")
        self.computer_vision_model_ = cv2.CascadeClassifier(self.computer_vision_model_filename_)

        # Subscribe to the camera topic
        self.image_sub_ = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size=1)

        self.marker_pub = rospy.Publisher('/artifact_localisation_marker', Marker, queue_size=1)

    
    def get_pose_2d(self):

        # Lookup the latest transform
        (trans,rot) = self.tf_listener_.lookupTransform('map', 'base_link', rospy.Time(0))

        # Return a Pose2D message
        pose = Pose2D()
        pose.x = trans[0]
        pose.y = trans[1]

        qw = rot[3];
        qz = rot[2];

        if qz >= 0.:
            pose.theta = wrap_angle(2. * math.acos(qw))
        else: 
            pose.theta = wrap_angle(-2. * math.acos(qw));

        print("pose: ", pose)

        return pose

    def LaserCallback(self, laserdata):
        self.laserData = laserdata


    def image_callback(self, image_msg):
        # This method is called when a new RGB image is received
        # Use this method to detect artifacts of interest
        #
        # A simple method has been provided to begin with for detecting stop signs (which is not what we're actually looking for) 
        # adapted from: https://www.geeksforgeeks.org/detect-an-object-with-opencv-python/

        # Copy the image message to a cv image
        # see http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        yolo, yolodetections = self.detector.process_image(image_msg)
        image = self.cv_bridge_.imgmsg_to_cv2(yolo, desired_encoding='passthrough')
        
        # Create a grayscale version, since the simple model below uses this
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Retrieve the pre-trained model
        stop_sign_model = self.computer_vision_model_

        # Detect artifacts in the image
        # The minSize is used to avoid very small detections that are probably noise
        detections = stop_sign_model.detectMultiScale(image, minSize=(20,20))
        #print("Yolo detections:", yolodetections)
        #print("Normal detections:", detections)
        # You can set "artifact_found_" to true to signal to "main_loop" that you have found a artifact
        # You may want to communicate more information
        # Since the "image_callback" and "main_loop" methods can run at the same time you should protect any shared variables
        # with a mutex
        # "artifact_found_" doesn't need a mutex because it's an atomic
        num_detections = len(yolodetections)

        if num_detections > 0:
            self.artifact_found_ = True
            self.generateGoals(yolodetections)
        else:
            self.artifact_found_ = False

        # Draw a bounding box rectangle on the image for each detection

        image_copy = image.copy()

        for(x, y, width, height) in detections:
            cv2.rectangle(image_copy, (x, y), (x + height, y + width), (0, 255, 0), 5)

        # Publish the image with the detection bounding boxes
        image_detection_message = self.cv_bridge_.cv2_to_imgmsg(image_copy, encoding="bgr8")

        self.image_detections_pub_.publish(image_detection_message)
        
        rospy.loginfo('image_callback')
        rospy.loginfo('artifact_found_: ' + str(self.artifact_found_))


    #connor add
    def generateGoals(self, detectionsArray):
        odom = self.get_pose_2d()

        laser = self.laserData

        for detection in detectionsArray:
            classID = detection[2]
            depth = detection[0] #in metres
            angle = detection[1] #in degrees
            angle = -1 * angle * math.pi / 180 #convert to radians and flip direction reading

            print("Object ID: ", classID)
            #turn depth + angle into a point using odom

            theta = odom.theta + angle
            if theta < 0:
                theta = theta + 2 * math.pi
            elif theta > 2 * math.pi:
                theta = theta - 2 * math.pi
            
            badReading = False

            if depth < 0.5 or depth > 7:
                badReading = True

            # see if the angle reading at theta matches :D
            i = 0
            for scan in laser.ranges:
                laserAngle = laser.angle_min + (i * laser.angle_increment)
                i += 1

                if abs(laserAngle - theta) < laser.angle_increment:
                    #less than increment gives an approximate closest answer(s)
                    if abs(scan - depth) > 2:
                        #if the laserscan range and perceived depth are too far apart, the reading is bad. 
                        badReading = True
                        

            if math.isnan(depth) == False and badReading == False:

                plannedGoalPos = Pose2D()
                if depth > 1:
                    depth = depth - 1

                

                #print("theta: ", theta, "yaw: ", odom.theta, "camAngle: ", angle, "depth: ", depth) debbugging

                plannedGoalPos.x = depth * math.cos(theta) + odom.x
                plannedGoalPos.y = depth * math.sin(theta) + odom.y
                plannedGoalPos.theta = theta

                alreadyVisited = False

                classCount = 0
                for node in self.artifactNodes:
                    dist = math.sqrt(pow(node.x - plannedGoalPos.x, 2) + pow(node.y - plannedGoalPos.y, 2))
                    if dist < 10 and self.artifactClasses[classCount] == classID:
                        alreadyVisited = True
                    classCount += 1

                #cancel current goal pathing if unvisited planned goal and 
                if alreadyVisited == False:

                    alreadyExists = False
                    existsID = 0
                    for i in range(len(self.artifactMaybe)):
                        dist = math.sqrt(pow(self.artifactMaybe[i].x - plannedGoalPos.x, 2) + pow(self.artifactMaybe[i].y - plannedGoalPos.y, 2))
                        if dist < 1.5:
                            alreadyExists = True
                            existsID = i

                    if alreadyExists:
                        self.artifactMaybeCounts[existsID] += 1
                        if self.artifactMaybeCounts[existsID] >= 10:
                            print("Made artifact at: ", plannedGoalPos.x, plannedGoalPos.y)
                            self.artifactNodes.append(plannedGoalPos)
                            self.artifactUnvisited.append(plannedGoalPos)
                            self.artifactClasses.append(classID)

                    else:
                        self.artifactMaybe.append(plannedGoalPos)
                        self.artifactMaybeCounts.append(1)

    def planner_move_forwards(self, action_state):
        # Simply move forward by 10m

        # Only send this once before another action
        if action_state == actionlib.GoalStatus.LOST:

            pose_2d = self.get_pose_2d()

            rospy.loginfo('Current pose: ' + str(pose_2d.x) + ' ' + str(pose_2d.y) + ' ' + str(pose_2d.theta))

            # Move forward 10m
            pose_2d.x += 10 * math.cos(pose_2d.theta)
            pose_2d.y += 10 * math.sin(pose_2d.theta)

            rospy.loginfo('Target pose: ' + str(pose_2d.x) + ' ' + str(pose_2d.y) + ' ' + str(pose_2d.theta))

            # Send a goal to "move_base" with "self.move_base_action_client_"
            action_goal = MoveBaseActionGoal()
            action_goal.goal.target_pose.header.frame_id = "map"
            action_goal.goal_id = self.goal_counter_
            self.goal_counter_ = self.goal_counter_ + 1
            action_goal.goal.target_pose.pose = pose2d_to_pose(pose_2d)

            rospy.loginfo('Sending goal...')
            self.move_base_action_client_.send_goal(action_goal.goal)



    def planner_go_to_first_artifact(self, action_state):
        # Go to a pre-specified artifact (alien) location

        # Only send this if not already going to a goal
        if action_state != actionlib.GoalStatus.ACTIVE:

            # Select a pre-specified goal location
            pose_2d = Pose2D()
            pose_2d.x = 18.0
            pose_2d.y = 25.0
            pose_2d.theta = -math.pi/2

            # Send a goal to "move_base" with "self.move_base_action_client_"
            action_goal = MoveBaseActionGoal()
            action_goal.goal.target_pose.header.frame_id = "map"
            action_goal.goal_id = self.goal_counter_
            self.goal_counter_ = self.goal_counter_ + 1
            action_goal.goal.target_pose.pose = pose2d_to_pose(pose_2d)

            rospy.loginfo('Sending goal...')
            self.move_base_action_client_.send_goal(action_goal.goal)



    def planner_return_home(self, action_state):
        # Go to the origin

        # Only send this if not already going to a goal
        if action_state != actionlib.GoalStatus.ACTIVE:

            # Select a pre-specified goal location
            pose_2d = Pose2D()
            pose_2d.x = 0
            pose_2d.y = 0
            pose_2d.theta = 0

            # Send a goal to "move_base" with "self.move_base_action_client_"
            action_goal = MoveBaseActionGoal()
            action_goal.goal.target_pose.header.frame_id = "map"
            action_goal.goal_id = self.goal_counter_
            self.goal_counter_ = self.goal_counter_ + 1
            action_goal.goal.target_pose.pose = pose2d_to_pose(pose_2d)

            rospy.loginfo('Sending goal...')
            self.move_base_action_client_.send_goal(action_goal.goal)

    def planner_random_walk(self, action_state):
        # Go to a random location, which may be invalid

        min_x = -5
        max_x = 50
        min_y = -5
        max_y = 50

        # Only send this if not already going to a goal
        if action_state != actionlib.GoalStatus.ACTIVE:

            # Select a random location
            pose_2d = Pose2D()
            pose_2d.x = random.uniform(min_x, max_x)
            pose_2d.y = random.uniform(min_y, max_y)
            pose_2d.theta = random.uniform(0, 2*math.pi)

            # Send a goal to "move_base" with "self.move_base_action_client_"
            action_goal = MoveBaseActionGoal()
            action_goal.goal.target_pose.header.frame_id = "map"
            action_goal.goal_id = self.goal_counter_
            self.goal_counter_ = self.goal_counter_ + 1
            action_goal.goal.target_pose.pose = pose2d_to_pose(pose_2d)

            rospy.loginfo('Sending goal...')
            self.move_base_action_client_.send_goal(action_goal.goal)

    def planner_random_goal(self, action_state):
        # Go to a random location out of a predefined set

        # Hand picked set of goal locations
        random_goals = [[53.3,40.7],[44.4, 13.3],[2.3, 33.4],[9.9, 37.3],[3.4, 18.5],[6.0, 0.4],[28.3, 11.8],[43.7, 12.8],[38.9,43.0],[47.4,4.7],[31.5,3.2],[36.6,32.5]]

        # Only send this if not already going to a goal
        if action_state != actionlib.GoalStatus.ACTIVE:

            # Select a random location
            idx = random.randint(0,len(random_goals)-1)
            pose_2d = Pose2D()
            pose_2d.x = random_goals[idx][0]
            pose_2d.y = random_goals[idx][1]
            pose_2d.theta = random.uniform(0, 2*math.pi)

            # Send a goal to "move_base" with "self.move_base_action_client_"
            action_goal = MoveBaseActionGoal()
            action_goal.goal.target_pose.header.frame_id = "map"
            action_goal.goal_id = self.goal_counter_
            self.goal_counter_ = self.goal_counter_ + 1
            action_goal.goal.target_pose.pose = pose2d_to_pose(pose_2d)

            rospy.loginfo('Sending goal...')
            self.move_base_action_client_.send_goal(action_goal.goal)

    def intersection_explore(self, actionstate):
        
        #The idea:
        #Traverse until reaching an intersection, which will be marked as a 'node'
        #Turn right, then continue until the next intersection.
        #If a previous intersection or a dead end, find an unvisited direction and go that way
        #Once no more unvisited directions the map should be complete.
        
        #Important parameters will be node placement tolerance, detection, and navigation
        #Also important to keep track of nodes and their branches.
        
        
        if actionstate != actionlib.GoalStatus.ACTIVE:
            #if not already going to goal -> launch into our intersection sweep
            self.nodes.CreateNodes(self.laserData, self.get_pose_2d(), self.FirstScan)
            
            if self.FirstScan == True:
                self.FirstScan = False
            #print("Length of unvisited set and all nodes: ", self.nodes.Unvisisted, self.nodes.AllNodes)
            closestNode = Node()
            closestDist = 999999
            
            for node in self.nodes.Unvisisted:
                #find the furthest unvisited node from robotPos and travel to it
                robotPose = self.get_pose_2d()
                dist = math.sqrt(pow(robotPose.x - node.x, 2) + pow(robotPose.y - node.y, 2))
                print("distance to goal: ", dist)
                if dist < closestDist:
                    closestNode = node
                    closestDist = dist
            
            #Move node to visited set
            if closestNode in self.nodes.Unvisisted:
                self.nodes.Visited.append(closestNode)
                self.nodes.Unvisisted.remove(closestNode)
    
                #Turn node into pose then into a goal
            nodePose = self.nodes.NodeToPose(closestNode)
            
            goal = MoveBaseActionGoal()
            goal.goal.target_pose.header.frame_id = "map"
            goal.goal_id = self.goal_counter_
            self.goal_counter_ = self.goal_counter_ + 1
            goal.goal.target_pose.pose = pose2d_to_pose(nodePose)
            
            #send goal to robot
            self.currGoal = goal
            self.move_base_action_client_.send_goal(goal.goal) 
        
    def go_artifact(self, action_state):
        if self.goingToArtifact == False:
            #travel to the closest artifact
            self.goingToArtifact = True
            self.move_base_action_client_.cancel_goal()

            odom = self.get_pose_2d()
            
            closestPoint = self.artifactUnvisited[0]
            closestDist = math.sqrt(pow(closestPoint.x - odom.x, 2) + pow(closestPoint.y - odom.y, 2))
            

            for point in self.artifactUnvisited[1:]:
                dist = math.sqrt(pow(point.x - odom.x, 2) + pow(point.y - odom.y, 2))
                if dist < closestDist:
                    closestDist = dist
                    closestPoint = point
            
            self.artifactUnvisited.remove(closestPoint)

            artifactGoal = MoveBaseActionGoal()
            artifactGoal.goal.target_pose.header.frame_id = "map"
            artifactGoal.goal_id = self.goal_counter_
            self.goal_counter_ = self.goal_counter_ + 1
            artifactGoal.goal.target_pose.pose = pose2d_to_pose(closestPoint)

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "artifact_markers"
            marker.id = self.MarkerCount # increment the marker ID
            marker.type = 3
            marker.action = 0 #change to 0
            marker.pose.position.x = closestPoint.x
            marker.pose.position.y = closestPoint.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
    
            # Publish the marker
            print("Made a marker")
            self.marker_pub.publish(marker)

            self.MarkerCount += 1


            self.move_base_action_client_.send_goal(artifactGoal.goal)

        else:
            if action_state == actionlib.GoalStatus.SUCCEEDED:
                #wait for 4 seconds
                rospy.sleep(2) #scary code
                self.goingToArtifact = False
                self.move_base_action_client_.send_goal(self.currGoal.goal)
            elif action_state == actionlib.GoalStatus.ABORTED:
                self.goingToArtifact = False
                self.move_base_action_client_.send_goal(self.currGoal.goal)

        
    

    def TSP_Solver(self):
        #go through all permutations, find the shortest loop, return it
        pi = math.pi

        Xs = [5, 17.5, 3.5, 18.5, 28, 38, 49, 34, 38, 52] #13.5, 14, 
        Ys = [20.5, 39, 35, 25, 12, 6, 4.5, 27, 44.5, 38.5] #1, 10.5, 
        Ts = [-pi/4, 0, -pi/2, 0, pi/2, 0, -pi/2, -pi/2, 0, pi/2] #0, -3*pi/4, 
        #points are in pose2D form, want 10-20
        goalArray = []

        #append a bunch of goal positions
        for i in range(len(Xs)):
            goalArray.append(Pose2D(Xs[i], Ys[i], Ts[i]))

        shortestDistance = 999999999
        shortestGoals = []
        #generate a permutation, find the total distance, if shortest move on
        
        print("Added goals to TSP")

        for permutation in permutations(goalArray):
            currOrder = []
            currOrder.append(Pose2D(0, 0, 0)) #first point always fixed
            for g in permutation:
                currOrder.append(g)
            currOrder.append(Pose2D(0, 0, 0))
            dist = self.TotalDistance(currOrder) #last point always fixed
            if dist < shortestDistance:
                shortestDistance = dist
                shortestGoals = currOrder
                print("Updated shortest dist: ", shortestDistance)
                #print("Best order so far: ", shortestGoals)
        print("Done finding goals")
        print(shortestGoals)
        return shortestGoals

    def TotalDistance(self, goalArray):
        #return the sum of distances

        distSum = 0

        prevGoal = goalArray[0]
        for goal in goalArray[1:]:
            dist = math.sqrt(pow(goal.x - prevGoal.x, 2) + pow(goal.y - prevGoal.y, 2))
            distSum += dist
            prevGoal = goal
        
        finalGoal = goalArray[0]
        finalDist = math.sqrt(pow(prevGoal.x - finalGoal.x, 2) + pow(prevGoal.y - finalGoal.y, 2))

        distSum += finalDist

        return distSum

    def Patrol(self, action_state):
        #go to each goal in order then loop

        #use self.idealGoals and self.patrolCounter
        if action_state != actionlib.GoalStatus.ACTIVE:
            
            goalPos = self.idealGoalOrder[self.patrolCounter - 1]

            self.patrolCounter += 1
            if self.patrolCounter >= len(self.idealGoalOrder):
                self.patrolCounter = 1

            #make goal, push goal

            Goal = MoveBaseActionGoal()
            Goal.goal.target_pose.header.frame_id = "map"
            Goal.goal_id = self.goal_counter_
            self.goal_counter_ = self.goal_counter_ + 1
            Goal.goal.target_pose.pose = pose2d_to_pose(goalPos)

            self.move_base_action_client_.send_goal(Goal.goal) 

        pass

    def main_loop(self):

        while not rospy.is_shutdown():

            #######################################################
            # Get the current status
            # See the possible statuses here: https://docs.ros.org/en/noetic/api/actionlib_msgs/html/msg/GoalStatus.html
            action_state = self.move_base_action_client_.get_state()
            rospy.loginfo('action state: ' + self.move_base_action_client_.get_goal_status_text())
            rospy.loginfo('action_state number:' + str(action_state))

            #if (self.planner_type_ == PlannerType.GO_TO_FIRST_ARTIFACT) and (action_state == actionlib.GoalStatus.SUCCEEDED):
            #    print("Successfully reached first artifact!")
            #    self.reached_first_artifact_ = True
            #if (self.planner_type_ == PlannerType.RETURN_HOME) and (action_state == actionlib.GoalStatus.SUCCEEDED):
            #    print("Successfully returned home!")
            #    self.returned_home_ = True




            #######################################################
            # Select the next planner to execute
            # Update this logic as you see fit!
            # self.planner_type_ = PlannerType.MOVE_FORWARDS
            
            #self.planner_type_ = PlannerType.TSP

            if len(self.artifactUnvisited) > 0 or self.goingToArtifact:
                self.planner_type_ = PlannerType.GO_TO_ARTIFACT
            else:
                self.planner_type_ = PlannerType.INTERSECTION_EXPLORE


            #######################################################
            # Execute the planner by calling the relevant method
            # The methods send a goal to "move_base" with "self.move_base_action_client_"
            # Add your own planners here!
            print("Calling planner:", self.planner_type_.name)
            if self.planner_type_ == PlannerType.MOVE_FORWARDS:
                self.planner_move_forwards(action_state)
            elif self.planner_type_ == PlannerType.GO_TO_FIRST_ARTIFACT:
                self.planner_go_to_first_artifact(action_state)
            elif self.planner_type_ == PlannerType.RETURN_HOME:
                self.planner_return_home(action_state)
            elif self.planner_type_ == PlannerType.RANDOM_WALK:
                self.planner_random_walk(action_state)
            elif self.planner_type_ == PlannerType.RANDOM_GOAL:
                self.planner_random_goal(action_state)
            elif self.planner_type_ == PlannerType.INTERSECTION_EXPLORE:
                self.intersection_explore(action_state)
            elif self.planner_type_ == PlannerType.GO_TO_ARTIFACT:
                self.go_artifact(action_state)
            elif self.planner_type_ == PlannerType.TSP:
                self.Patrol(action_state)


            #######################################################
            # Delay so the loop doesn't run too fast
            rospy.sleep(0.2)



if __name__ == '__main__':

    # Create the ROS node
    rospy.init_node('cave_explorer')

    # Create the cave explorer
    cave_explorer = CaveExplorer()

    # Loop forever while processing callbacks
    cave_explorer.main_loop()