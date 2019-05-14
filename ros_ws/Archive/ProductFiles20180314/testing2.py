import rospy
import baxter_interface as baxter
from sensor_msgs.msg import Image
import time
import cv2
import cv_bridge

import scanForMarker

rospy.init_node('imageListener')

while not rospy.is_shutdown():
	scanForMarker.markerPos()

cv2.destroyAllWindows()