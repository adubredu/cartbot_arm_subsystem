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

"""
if not rGripper.calibrate():
    print("right gripper did not calibrate")
    sys.exit()
"""

pause_event = Event()

limbName = 'left'
kin = baxter_kinematics(limbName)
endOfOpen = {"left_w0":-1.0227816903225997,"left_w1":-1.4534467965214295,"left_w2":-2.5276168432381905,"left_e0":-0.6166602767299363,"left_e1":2.1184274680697563,"left_s0":0.27113110425874687,"left_s1":-1.1516360765049745}

tasks.getFoodFromMicrowave(lLimb, rLimb, lGripper, pause_event)