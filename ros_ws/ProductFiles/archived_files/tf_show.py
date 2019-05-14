#! /usr/bin/env python
import rospy
import tf2_ros
import tf

rospy.init_node('tfshow')
tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
tf_listener = tf2_ros.TransformListener(tf_buffer)

transform = tf_buffer.lookup_transform('base', 'camera_depth_optical_frame',rospy.Time(0),
	rospy.Duration(1.0))
print transform
print "\nEuler"
quaternion = (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w)
euler = tf.transformations.euler_from_quaternion(quaternion)
print euler