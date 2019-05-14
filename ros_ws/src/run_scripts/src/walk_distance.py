#! /usr/bin/env python
import rospy
import sys
import time
from walk_distance_msg.msg import Walk 

rospy.init_node('walk_node', log_level = rospy.INFO)
pub = rospy.Publisher('/walk', Walk, queue_size=10)

walk = Walk()
walk.direction = 2
walk.distance = 1.0
for i in range(2):
	pub.publish(walk)
	time.sleep(5)
	