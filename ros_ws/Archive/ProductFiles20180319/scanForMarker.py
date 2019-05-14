import rospy
import baxter_interface as baxter
from sensor_msgs.msg import Image
import time
import cv2
import cv_bridge
import numpy as np

global leftImage

def image(topicData):
	bridge = cv_bridge.CvBridge()
	global leftImage
	leftImage = bridge.imgmsg_to_cv2(topicData, desired_encoding="bgra8")

collisionSubs = rospy.Subscriber(name='/cameras/left_hand_camera/image', 
								 data_class=Image, callback=image, 
								 buff_size=100)

def markerPos():
	global leftImage
	hsv = cv2.cvtColor(leftImage, cv2.COLOR_BGR2RGB)
	hsv = cv2.flip(hsv, 0)
	hsv = cv2.flip(hsv, 1)

	lower_blue = cv2.scalar([0,200,220])
	upper_blue = cv2.scalar([0,220,240])

	mask = cv2.inRange(hsv, lower_blue, upper_blue)
	#mask = cv2.inRange(mask, 240, 255)
	iAve, jAve, tot = getWeightCenter(mask)
	mask[iAve, :] = 255
	mask[:, jAve] = 255
	cv2.imshow('mask', mask)
	cv2.waitKey(10)

def getWeightCenter(maskedIm):
	height, width = maskedIm.shape

	iSum = 0
	jSum = 0
	tot = 0
	for i in range(height):
		for j in range(width):
			if maskedIm[i, j] >= 240 :
				iSum += i
				jSum += j
				tot += 1

	if tot:
		iAve = float(iSum - 70000)/float(tot)
		jAve = float(jSum - 90000)/float(tot)
		return int(iAve), int(jAve), tot
	return 0, 0, tot;