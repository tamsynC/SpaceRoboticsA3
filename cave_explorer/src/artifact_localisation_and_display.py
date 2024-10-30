from marker_msgs.msg import Marker
import rospy
from std_msgs.msg import String

from object_dectection import ObjectDetector

class ArtifactLocalisationAndDisplay:

    def __init__(self):
        self.local_pub = rospy.Publisher('/artifact_localisation_marker', Marker, queue_size=10)

    def artifact_localisation(self):
        pass