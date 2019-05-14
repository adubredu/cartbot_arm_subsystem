#! /usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers

def print_poses(data):
	
	if not data.markers:
		return
	tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
	tf_listener = tf2_ros.TransformListener(tf_buffer)

	for i in range(0,len(data.markers)):
				if data.markers[i].id == 11:
					print "Frame_id: Base -----Marker id: 11"
					marker_pose = data.markers[i].pose
					transform1 = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
						rospy.Duration(1.0))
					goal_pose1 = tf2_geometry_msgs.do_transform_pose(marker_pose, transform1)
					print goal_pose1
					print "------------------------------------"

				if data.markers[i].id == 19:
					print "Frame_id: Base -----Marker id: 19"
					marker_pose = data.markers[i].pose
					transform2 = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
						rospy.Duration(1.0))
					goal_pose2 = tf2_geometry_msgs.do_transform_pose(marker_pose, transform2)
					print goal_pose2
					print "**************************************"

if __name__=='__main__':
	rospy.init_node('print_pose')
	try:
		sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, print_poses)
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass