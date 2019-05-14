import rospy as ros
import baxter_interface as baxter
import json
import positionControl as pos
import taskFunctions as tasks
import time
from threading import *
from baxter_pykdl import baxter_kinematics
import ctypes


def terminate_thread(thread):
    """Terminates a python thread from another thread.

    :param thread: a threading.Thread instance
    """
    if not thread.isAlive():
        return

    exc = ctypes.py_object(KeyboardInterrupt)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(thread.ident), exc)
    if res == 0:
        raise ValueError("nonexistent thread id")
    elif res > 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


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

lGripper.set_holding_force(100)
lGripper.set_moving_force(100)

rGripper.set_holding_force(100)
rGripper.set_moving_force(100)

"""
if not rGripper.calibrate():
    print("right gripper did not calibrate")
    sys.exit()
"""

pause_event = Event()

tasks.moveToDownward(lLimb, rLimb, pause_event)
tasks.pickUpBottleFromTable(lLimb, rLimb, lGripper, pause_event)
tasks.putBottleInHolder(lLimb, rLimb, lGripper, pause_event)