#!/usr/bin/env python
import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from duckietown_msgs.msg import AprilTagDetection 
from duckietown_msgs.msg import AprilTagDetectionArray

class ObstacleDetectorNode(object):
	def __init__(self):
		self.node_name = "obstacle_detector_node"
		self.sub_image = rospy.Subscriber("image_rect", Image, self.cbImage, queue_size=1)
		self.pub_visualize = rospy.Publisher("~tag_detections", AprilTagDetectionArray, queue_size=1)

	def cbImage(self, image):
		placeholder_z = 10
		placeholder_tag_id = 0

		tag_detection_array = AprilTagDetectionArray()
		tag_pose = PoseStamped()
		tag_pose.pose.position.z = placeholder_z
		tag_detection = AprilTagDetection()
		tag_detection.pose = tag_pose
		tag_detection.id = placeholder_tag_id
		tag_detection_array.detections.append(tag_detection)
		self.pub_visualize.publish(tag_detection_array)



if __name__ == "__main__":
	rospy.init_node('obstacle_detector_node',anonymous=False)
	node = ObstacleDetectorNode()
	rospy.spin()