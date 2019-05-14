import rospy as ros
import baxter_interface as baxter
import json
import positionControl as pos
import taskFunctions as tasks
import time

ros.init_node('cartRecorder')

baxter_enabler = baxter.RobotEnable(versioned=True)
baxter_enabler.enable()

lLimb = baxter.Limb('left')
rLimb = baxter.Limb('right')
lGripper = baxter.Gripper('left')
rGripper = baxter.Gripper('right')

# calibrating gripper
if not lGripper.calibrate():
    print("left gripper did not calibrate")
    sys.exit()

if not rGripper.calibrate():
    print("right gripper did not calibrate")
    sys.exit()

# fName = '{}.wp'.format(raw_input('Type file name: '))
#tasks.openFridge(lLimb, rLimb)
tasks.getFoodContainer(lLimb, rLimb, lGripper)
time.sleep(2)
tasks.placeFoodContainerInMicrowave(lLimb, rLimb, lGripper)