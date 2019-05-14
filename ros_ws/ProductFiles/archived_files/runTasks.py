from multiprocessing import Process
from threading import (
    Thread,
    activeCount,
    Event as TEvent,
    enumerate,
)

import ctypes
import rospy
from std_msgs.msg import (
    UInt16,
    Bool,
)
import baxter_interface as baxter
from positionControl import *
from mobileTasks import *

#from baxterGUI import lLimb, rLimb, lGripper, rGripper, lastAliveName

# Helper functions #
def terminate_thread(thread):
    if not thread.isAlive():
        return
    exc = ctypes.py_object(KeyboardInterrupt)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
    ctypes.c_long(thread.ident), exc)
    if res == 0:
        raise ValueError("nonexistent thread id")
    elif res > 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def runTask(task, pause_event, task_target, task_name, args):
    #global task, lLimb, rLimb, pause_event, rawCommand
    task = Thread(target=task_target, args=args,
                  name=task_name)
    print(task.name)
    pause_event.clear()
    task.daemon = True
    task.start()
    #update_command_box(task_name)
    rawCommand = ""


def directionalTasks(command,lLimb, lGripper, lastAliveName, task, pause_event):
    ##### Execute relevant command #####
    #### Directional Commands #####
    if 'move arm forward' in command:
        terminate_thread(task)
        args=(lLimb, 'y', 4, speed, pause_event)
        runTask(task, pause_event,moveOnAxis, "movingForward", args)
    if 'move arm backward' in command:
        terminate_thread(task)
        args=(lLimb, 'y', -4, speed, pause_event)
        runTask(task, pause_event, moveOnAxis, "movingBackward", args)
    if 'move arm left' in command:
        terminate_thread(task)
        args=(lLimb, 'x', -4, speed, pause_event)
        runTask(task, pause_event,moveOnAxis, "movingLeft", args)
    if 'move arm right' in command:
        terminate_thread(task)
        args=(lLimb, 'x', 4, speed, pause_event)
        runTask(task, pause_event,moveOnAxis, "movingRight", args)
    if  'arm' in command and ('higher' in command or 'up' in command):
        terminate_thread(task)
        args=(lLimb, 'z', 4, speed, pause_event)
        runTask(task, pause_event,moveOnAxis, "movingHigher", args)
    if  'arm' in command and ('lower' in command or 'down' in command):
        terminate_thread(task)
        args=(lLimb, 'z', -4, speed, pause_event)
        runTask(task, pause_event,moveOnAxis, "movingLower", args)
    ### End Directional Commands ###

    ### Gripper Direct Commands ###
    if 'close' in command and ('hand' in command or 'gripper' in command):
        terminate_thread(task)
        lGripper.close()
        rawCommand = ""
    if 'open' in command and ('hand' in command or 'gripper' in command):
        terminate_thread(task)
        lGripper.open()
        rawCommand = ""
    ### End Gripper Direct Commands ###


# what didn't work - keep runTasks in main file and just supply arguements here...
#giant ugly function
def identifyTask(command, lLimb, rLimb, lGripper, rGripper,lastAliveName, task, pause_event,env):
### Begin fridge commands ###
# Open the fridge #
    rawCommand = command
    if 'open' in command and 'the fridge' in command:
        terminate_thread(task)
        if not env['fridgeOpen']:
            args=(lLimb, rLimb, pause_event)
            runTask(task, pause_event,openFridge, "openingFridge", args)

# Get waterbottle from fridge #
    if 'get' in command and 'water bottle' in command:
        terminate_thread(task)
        # if fridge is closed #
        if not env['fridgeOpen'] and not env['hasBottle']:
            args = (lLimb, rLimb,lGripper, pause_event)
            runTask(task, pause_event,getBottleFromStart, "gettingBottleFromStart", args)
            # if fridge is open #
        if env['fridgeOpen'] and not env['hasBottle']:
            args=(lLimb, rLimb,lGripper, pause_event)
            runTask(task, pause_event,pickBottleFromOpenFridge, "gettingBottleFromOpenFridge",args)
            # if fridge is open and has bottle #
        if env['fridgeOpen'] and env['hasBottle']:
            rawCommand = ""
        
# Close the fridge #
    if 'close' in command and 'the fridge' in command:
        terminate_thread(task)
        if env['fridgeOpen'] and not env['holdingSomething']:
            args=(lLimb, rLimb, pause_event)
            runTask(task, pause_event,closeFridge, "closingFridge", args)     

# Place water bottle on table #
    if 'place' in command and 'table' in command:
        terminate_thread(task)
        if env['hasBottle']:
            args=(lLimb, rLimb,lGripper, pause_event)
            runTask(task, pause_event,moveToTableAfterRetrieve, "placingOnTable", args)
        else:
            print("I don't have the bottle right now")
            print(task.name)
            rawCommand = ""

# Full command - Put water bottle on table #
    if 'put' in command and 'water bottle' in command and 'table' in command:
        terminate_thread(task)
        args=(lLimb, rLimb,lGripper,pause_event)
        # if the fridge is closed #
        if not env['fridgeOpen'] and not env['hasBottle']:
            runTask(task, pause_event,getBottleFull, "gettingBottlePlacingOnTable", args)
        # if the fridge is open #
        elif env['fridgeOpen'] and not env['hasBottle']:
            runTask(task, pause_event,bottleOnTableAfterOpenFridge,"gettingBottleFromOpenFridgeAndPlacingOnTable", args)
        # if it has water bottle already #
        elif env['fridgeOpen'] and env['hasBottle']:
            runTask(task, pause_event,moveToTableAfterRetrieve, "placingOnTable", args)
#### End Fridge task commands ####

##########################  
### Microwave commands ###
##########################
#Open the microwave#
    if 'open' in command and 'the microwave' in command:
        terminate_thread(task)
        if not env["microwaveOpen"] and not env["holdingSomething"]:
            args=(lLimb, rLimb, pause_event)
            runTask(task, pause_event,openMicrowave, "openingMicrowave", args)
        rawCommand = ""

# Close the Microwave #
    if 'close' in command and 'microwave' in command:
        terminate_thread(task)
        if env["microwaveOpen"] and not env["holdingSomething"]:
            args=(lLimb, rLimb, pause_event)
            runTask(task, pause_event,closeMicrowave, "closingMicrowave", args)
        rawCommand = ""

# turn on microwave #    
    if 'start' in command and 'microwave' in command:
        terminate_thread(task)
        if (not env["microwaveOpen"]) and (not env["holdingSomething"]) and (not env["microwaveOn"]):
            args=(lLimb, rLimb, pause_event)
            runTask(task, pause_event,turnOnMicrowave, "turningOnMicrowave", args)
            env["microwaveOn"] = True
        rawCommand = ""

# turn off microwave # 
    if ('turn off' in command) or ('stop'in command and 'microwave' in command):
        terminate_thread(task)
        if env["microwaveOn"] and (not env["holdingSomething"]):
            args=(lLimb, rLimb, pause_event)
            runTask(task, pause_event,turnOffMicrowave, "turningOffMicrowave", args)
        rawCommand = ""

# timed cook #    
    if 'cook' in command and 'seconds' in command:
        terminate_thread(task)
        t = [int(s) for s in command.split() if s.isdigit()]
        print(t)
        if not t:
            print("No time given")
        elif (not env["microwaveOpen"]) and (not env["holdingSomething"]) and (not env["microwaveOn"]):
            args=(lLimb, rLimb, pause_event, t[0])
            runTask(task, pause_event,timedMicrowave, "timedCook", args)
            env["microwaveOn"] = True
        rawCommand = ""

# put food in microwave #
    if (('meal' in command) or ('food' in command)) and (('put' in command or 'place' in command) and ('microwave' in command)):
        terminate_thread(task)
        if not env["fridgeOpen"] and not env["microwaveOpen"] and not env["holdingSomething"] and not env["foodInMicrowave"]:
            args=(lLimb, rLimb, lGripper, pause_event,)
            runTask(task, pause_event,placeContainerInMicrowaveFromStart, "puttingFoodInMicrowave",args)
        if env["fridgeOpen"] and not env["microwaveOpen"] and not env["holdingSomething"] and not env["foodInMicrowave"]:
            args=(lLimb, rLimb, lGripper, pause_event,)
            runTask(task, pause_event,placeContainerInMicrowaveFromOpenFridge,"puttingFoodInMicrowave", args)
        if not env["fridgeOpen"] and env["microwaveOpen"] and not env["holdingSomething"] and not env["foodInMicrowave"]:
            args=(lLimb, rLimb, lGripper, pause_event,)
            runTask(task, pause_event,placeContainerInMicrowaveFromOpenMicrowave, "puttingFoodInMicrowave", args)
        if env["fridgeOpen"] and env["microwaveOpen"] and not env["holdingSomething"] and not env["foodInMicrowave"]:
            args=(lLimb, rLimb, lGripper, pause_event,)
            runTask(task, pause_event,placeContainerInMicrowaveFromOpenMicOpenFridge,"puttingFoodInMicrowave", args) 
        rawCommand = ""

# get food out of microwave #
    if ('take' in command or 'get' in command) and ('container' in command or 'food' in command) and ('microwave' in command):
        terminate_thread(task)
        if not env["microwaveOpen"] and env["foodInMicrowave"]:
            args=(lLimb, rLimb, lGripper, pause_event,)
            runTask(task, pause_event,getFoodFromMicrowave, "gettingFoodFromMicrowave", args)
        if env["microwaveOpen"] and env["foodInMicrowave"]:
            args=(lLimb, rLimb, lGripper, pause_event,)
            runTask(task, pause_event,getFoodFromOpenMicrowave, "gettingFoodFromMicrowave", args)
        rawCommand = ""
##end microwave Commands
### End task related commands ###
    return rawCommand