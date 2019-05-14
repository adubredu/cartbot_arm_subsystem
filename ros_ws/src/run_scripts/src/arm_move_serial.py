import rospy
import time
from geometry_msgs.msg import Twist
import serial
from run_scripts.srv import *

#On cartbotarm raspberry pi



class Arm_teleop:


        def arm_callback(self,data):
                if data.angular.z == 0 and data.linear.x == 0:
                        self.rotate.flush()
                        self.rotate.write('s')

                elif data.angular.z < 0:
                        self.rotate.flush()
                        self.rotate.write('f')

                elif data.angular.z > 0:
                        self.rotate.flush()
                        self.rotate.write('r')

                if data.linear.x > 0:
                        if not self.should_elevate:
                                self.elongate.flush()
                                self.elongate.write('f')

                        else:
                                self.elevate.flush()
                                self.elevate.write('f')

                elif data.linear.x < 0:
                        if not self.should_elevate:
                                self.elongate.flush()
                                self.elongate.write('r')
                        else:
                                self.elevate.flush()
                                self.elevate.write('r')

                elif data.angular.z == 0 and data.linear.x == 0:
                        self.elongate.flush()
                        self.elongate.write('s')
                        self.elevate.flush()
                        self.elevate.write('s')



	def handle_command(self, req):
		print "command received"
                command = req.command
                self.interprete_command(command)
                return CommandResponse(True)
 


        def interprete_command(self, command):
                if "elevate" in command:
                        self.should_elevate = True

                elif "extend" in command:
                        self.should_elevate = False
		elif "close" in command:
			self.gripper.flush()
			self.gripper.write('c')
		elif "open" in command:
			self.gripper.flush()
			self.gripper.write('o')


        def __init__(self):
                rospy.init_node('arm_mover')
                rospy.Subscriber('/arm_vel', Twist, self.arm_callback)
                self.rotate = serial.Serial('/dev/ttyACM0', 9600)
                self.elongate = serial.Serial('/dev/ttyACM1', 9600)
                self.elevate = serial.Serial('/dev/ttyACM2', 9600)
		self.gripper = serial.Serial('/dev/ttyACM3', 9600)  
                self.should_elevate = False

                s = rospy.Service('receive_command', Command, self.handle_command)

                rospy.spin()


if __name__=="__main__":
        try:
                a = Arm_teleop()

        except rospy.ROSInterruptException: pass
'''

                        else:
                                self.elevate.flush()
                                self.elevate.write('f')

                elif data.linear.x < 0:
                        if not self.should_elevate:
                                self.elongate.flush()
                                self.elongate.write('r')
                        else:
                                self.elevate.flush()
                                self.elevate.write('r')

                elif data.angular.z == 0 and data.linear.x == 0:
                        self.elongate.flush()
                        self.elongate.write('s')
                        self.elevate.flush()
                        self.elevate.write('s')



       def handle_command(self, req):
                command = req.command
                self.interprete_command(command)
                return CommandResponse(True)
 


        def interprete_command(self, command):
                if "elevate" in command:
                        self.should_elevate = True

                elif "rotate" in command:
                        self.should_elevate = False



        def __init__(self):
                rospy.init_node('arm_mover')
                rospy.Subscriber('/arm_vel', Twist, arm_callback)
                self.rotate = serial.Serial('/dev/ttyACM0', 9600)
                self.elongate = serial.Serial('/dev/ttyACM1', 9600)
                self.elevate = serial.Serial('/dev/ttyACM2', 9600)
               
                self.should_elevate = False

                s = rospy.Service('receive_command', Command, self.handle_command)

                rospy.spin()


if __name__=="__main__":
        try:
                a = Arm_teleop()

        except rospy.ROSInterruptException: pass
'''
