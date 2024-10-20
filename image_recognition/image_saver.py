# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

save_interval = 30
pic_num = 0

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    global pic_num
    # print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        pic_num += 1

        if pic_num % save_interval == 0:
            # Save your OpenCV2 image as a jpeg
            cv2.imwrite("pictures/" + str(pic_num) + ".png", cv2_img)
            print("saved " + str(pic_num))

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()