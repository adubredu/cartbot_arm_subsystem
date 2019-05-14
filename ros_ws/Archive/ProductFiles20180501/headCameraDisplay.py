import rospy
import baxter_interface as baxter
from sensor_msgs.msg import Image
import time
import cv2
import cv_bridge
import numpy as np

global headImage

def image(topicData):
	bridge = cv_bridge.CvBridge()
	global leftImage
	leftImage = bridge.imgmsg_to_cv2(topicData, desired_encoding="bgra8")
	cv2.imshow('Environment', leftImage)

cameraSubs = rospy.Subscriber(name='/cameras/head_camera/image', 
								 data_class=Image, callback=image, 
								 buff_size=100)