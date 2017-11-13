#!/usr/bin/env python
import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

class ObstacleDetectorNode(object):
	def __init__(self):
		self.node_name = "obstacle_detector_node"
		self.sub_image = rospy.Subscriber("image_rect", Image, self.cbImage, queue_size=1)
		self.pub_visualize = rospy.Publisher("~tag_detections", PoseStamped, queue_size=1)

	def cbImage(self):
		pass



if __name__ == "__main__":
	rospy.init_node('obstacle_detector_node',anonymous=False)
	node = ObstacleDetectorNode()
	rospy.spin()