import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO  # Assuming you use YOLOv8 or later versions from ultralytics

class ObjectDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('object_detector_node', anonymous=True)

        # Initialize the YOLO model (if needed)
        self.model = YOLO('YOLO.pt')  # Load YOLOv8 small model (adjust if needed)

        # Initialize a CvBridge object for converting ROS images to OpenCV images and vice versa
        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        # Publisher for republishing the same image
        self.image_pub = rospy.Publisher("/yolo/processed_image", Image, queue_size=10)

    def image_callback(self, image_msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Optionally, run YOLO detection here (currently not processing)
            results = self.model(cv_image)
            # annotated_image = results.render()  # Optionally annotate the image with detection boxes

            # # Convert the image (original or processed) back to ROS Image message
            # output_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

            annotated_image = results[0].plot()  # Use plot() to annotate image with bounding boxes

            # Convert the processed image back to ROS Image message
            output_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")



            # Publish the image to the new topic
            self.image_pub.publish(output_image_msg)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridgeError: {e}")

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        # Create an instance of ObjectDetector and run it
        detector = ObjectDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
