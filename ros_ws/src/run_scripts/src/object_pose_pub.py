#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from os.path import expanduser
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32
import cv2
import numpy as np 

'''
blue - cup
red - bowl
green - bottle
good
greenlb = np.array([36,60,40])
greenub = np.array([90,255,255])

bluelb = np.array([90,200,50])
blueub = np.array([130,255,255])


redlb = np.array([0,60,20])
redub = np.array([5,255,255])


'''
class Floor_Object_Pose:

	#Locates the object and returns its position and orientation
	def detect_object(self,img):
		#upper and lower bounds for HSV for Green color
		greenlb = np.array([36,60,40])
		greenub = np.array([90,255,255])

		lower_blue = np.array([100,150,0])
		upper_blue = np.array([140,255,255])

		bluelb = np.array([90,200,50])
		blueub = np.array([130,255,255])

		redlb = np.array([0,0,0])
		redub = np.array([180,255,10])

		kernelOpen = np.ones((5,5))
		kernelClose = np.ones((20,20))

		#convert BGR to HSVV
		imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		#Create mask
		greenmask = cv2.inRange(imgHSV,greenlb,greenub)
		bluemask = cv2.inRange(imgHSV,bluelb,blueub)
		# blackmask = cv2.inRange(imgHSV,blacklb,blackub)
		# redmask = cv2.inRange(imgHSV,redlb,redub)

		#morphology
		#Morph open removes stray white pixels
		#Morph close removes stray black pixels in detected object
		maskOpen = cv2.morphologyEx(greenmask, cv2.MORPH_OPEN, kernelOpen)
		greenmask= cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)

		maskOpen = cv2.morphologyEx(bluemask, cv2.MORPH_OPEN, kernelOpen)
		bluemask= cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)

		# maskOpen = cv2.morphologyEx(redmask, cv2.MORPH_OPEN, kernelOpen)
		# redmask= cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)

		# maskOpen = cv2.morphologyEx(redmask, cv2.MORPH_OPEN, kernelOpen)
		# redmask= cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)
		greenrect = None; bluerect=None; 

		_,gcnts, _ = cv2.findContours(greenmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		gcnts = sorted(gcnts, key=cv2.contourArea, reverse=True)
		if len(gcnts):    
			c = gcnts[0]      
			greenrect = cv2.minAreaRect(c)

		_,bcnts, _ = cv2.findContours(bluemask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		bcnts = sorted(bcnts, key=cv2.contourArea, reverse=True)
		if len(bcnts):    
			c = bcnts[0]      
			bluerect = cv2.minAreaRect(c)


			
		return  greenrect, bluerect




#publishes the transform of the 3D pose of object to the frame /camera_link
	def run(self):

		#while ROS is still running
		while not rospy.is_shutdown():
			if self.current_image is not None:
				try:
					#convert the sensor_msg/Image to a cv2 image
					self.scene = self.bridge.imgmsg_to_cv2(self.current_image, 'bgr8')

					#gets the position and 2D orientation of the objet in the image as well
					#as the original image with a rectangle around the detected object
					green, blue = self.detect_object(self.scene)

					#Publishes the annotated image onto the /floor_object/image topic
					# self.imagepub.publish(self.bridge.cv2_to_imgmsg(self.scene, 'rgb8'))
					# cv2.imshow('Test',self.scene)
					# if cv2.waitKey(1) &0xFF == ord('q'):
					#     break



					#locates the position of the center of the detected object
					#in its image frame.
					#It uses the coordinates to determine the 3D position of the 
					#point cloud representing the coordinates of the center.
					#It then publishes a transform between the object and the camera frame
					#and publishes the pose of the object
					
					if green is not None:
						center = green[0]
						if self.current_pc is None:
							rospy.loginfo('No point cloud information available')

						else:
							pc_list = list(pc2.read_points(self.current_pc, skip_nans=True, field_names=('x', 'y', 'z'), uvs=[(int(center[0]), int(center[1]))]))

							if len(pc_list) > 0:
								tf_id = 'bottle'
								point_x, point_y, point_z = pc_list[0]

								object_tf = [point_z, -point_x, -point_y]
								frame = '/camera_link'

								quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)#self.orientation)

								self.tfpub.sendTransform((object_tf), quaternion, rospy.Time.now(), tf_id, frame)
								

								object_pose = PoseStamped()
								object_pose.header.stamp = rospy.Time.now()
								object_pose.header.frame_id = frame
								object_pose.pose.position.x = object_tf[0]
								object_pose.pose.position.y = object_tf[1]
								object_pose.pose.position.z = object_tf[2]
								
								object_pose.pose.orientation.x = quaternion[0]
								object_pose.pose.orientation.y = quaternion[1]
								object_pose.pose.orientation.z = quaternion[2]
								object_pose.pose.orientation.w = quaternion[3]

								self.bottlepub.publish(object_pose)
					

					if blue is not None:
						center = blue[0]
						if self.current_pc is None:
							rospy.loginfo('No point cloud information available')

						else:
							pc_list = list(pc2.read_points(self.current_pc, skip_nans=True, field_names=('x', 'y', 'z'), uvs=[(int(center[0]), int(center[1]))]))

							if len(pc_list) > 0:
								tf_id = 'cup'
								point_x, point_y, point_z = pc_list[0]

								object_tf = [point_z, -point_x, -point_y]
								frame = '/camera_link'

								quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)#self.orientation)

								self.tfpub.sendTransform((object_tf), quaternion, rospy.Time.now(), tf_id, frame)
								

								object_pose = PoseStamped()
								object_pose.header.stamp = rospy.Time.now()
								object_pose.header.frame_id = frame
								object_pose.pose.position.x = object_tf[0]
								object_pose.pose.position.y = object_tf[1]
								object_pose.pose.position.z = object_tf[2]
								
								object_pose.pose.orientation.x = quaternion[0]
								object_pose.pose.orientation.y = quaternion[1]
								object_pose.pose.orientation.z = quaternion[2]
								object_pose.pose.orientation.w = quaternion[3]

								self.cuppub.publish(object_pose)
	


				except CvBridgeError as e:
					print(e)
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
					print(e)



	def __init__(self):
		self.tf_listener = tf.TransformListener()
		self.bridge = CvBridge()
		self.current_image = None
		self.current_pc = None
		self.scene = None
		self.orientation = None
		self.tfpub = tf.TransformBroadcaster()
		rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
		rospy.Subscriber('/camera/depth/points', PointCloud2, self.pc_callback)

		self.bowlpub = rospy.Publisher('/bowl_pose', PoseStamped, queue_size=10)
		self.cuppub = rospy.Publisher('/cup_pose', PoseStamped, queue_size=10)
		self.bottlepub = rospy.Publisher('/bottle_pose', PoseStamped, queue_size=10)


	def image_callback(self, image):
		self.current_image = image


	def pc_callback(self, pc):
		self.current_pc = pc



if __name__ == '__main__':
	rospy.init_node('floor_object_pose_publisher', log_level = rospy.INFO)

	try:
		f = Floor_Object_Pose()
		f.run()
	except KeyboardInterrupt:
		rospy.loginfo('Shutting down')


#'''