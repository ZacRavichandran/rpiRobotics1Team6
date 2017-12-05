#!/usr/bin/env python
import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from duckietown_msgs.msg import AprilTagDetection 
from duckietown_msgs.msg import AprilTagDetectionArray
from sign_detector import find_stop_signs, find_ducks
import numpy as np
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
import cv2
import os 

class ObstacleDetectorNode(object):
	def __init__(self):
		self.node_name = "obstacle_detector_node"
		self.sub_image = rospy.Subscriber("~image_rect", Image, self.cbImage, queue_size=1)
		self.bridge = CvBridge()
		self.pub_visualize = rospy.Publisher("~tag_detections", AprilTagDetectionArray, queue_size=1)
		self.stop_sign_id = 10

	def write_results(self, file, shape, size, ar, r, g, b, stop_sign = True):
		msg = "%d,%f,%f,%f,%f,%f,%f\n" % (stop_sign, shape, size, ar, r, g, b)
		self.write_to_file(file, msg)

	def write_to_file(self, file, msg):
		dir_path = os.path.dirname(os.path.realpath(__file__))
		file_loc = dir_path + "/" + file
		with open(file_loc, 'a') as f:
			f.write(msg)		

	def find_signs(self, img):
		shapes = find_stop_signs(img)

		published = False
		for shape in shapes:
			(x, y, w, h) = cv2.boundingRect(shape.approx)
			ar = w / float(h)
			if shape.size > 100 and ar >= 0.3: # shape.shape >= 3 and shape.size > 1000 and ar <= 1.05 and ar >= 0.7:
				rospy.loginfo("Found shape: %d at (%dx%d) of size %d with %0.3f, rgb=(%d, %d, %d)" % \
					(shape.shape, shape.cx, shape.cy, shape.size, ar, shape.r, shape.g, shape.b))
				self.write_results("stop_sign_data_2.txt", shape.shape, shape.size, ar, shape.r, shape.g, shape.b, stop_sign = True)
				self.make_and_publish_position_message(0, 10)
				published = True

		# send signal nothing was found
		if published == False:
			self.make_and_publish_position_message(100, -1)
			rospy.loginfo("Found nothing")
			rospy.loginfo("x ====== x")
		# print output for debugging
		else:
			rospy.loginfo("<====>")

	def find_and_process_ducks(self, img):
		shapes = find_ducks(img)

		# size of ducks is typically > 1000
		for shape in shapes:
			if shape.shape >= 4 and shape.size > 1000:
				rospy.loginfo("Found potential duck of shape: %d at (%dx%d) of size %d" % (shape.shape, shape.cx, shape.cy, shape.size))
		print("<====>")

	def make_and_publish_position_message(self, msg_z, msg_id):
		tag_detection_array = AprilTagDetectionArray()
		tag_pose = PoseStamped()
		tag_pose.pose.position.z = msg_z
		tag_detection = AprilTagDetection()
		tag_detection.pose = tag_pose
		tag_detection.id = msg_id
		tag_detection_array.detections.append(tag_detection)
		self.pub_visualize.publish(tag_detection_array)


	def cbImage(self, image):
		#self.find_signs(np.array(image.data))
		cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
		np_image = np.array(cv_image)
		self.find_signs(np_image)
		#self.find_and_process_ducks(np_image)



if __name__ == "__main__":
	rospy.init_node('obstacle_detector_node',anonymous=False)
	node = ObstacleDetectorNode()
	rospy.spin()