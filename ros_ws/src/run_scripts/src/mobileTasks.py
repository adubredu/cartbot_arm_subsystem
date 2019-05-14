############# Baxter Speech Arm Positional Control ################

## Task Functions Package

## Written by the Robotic Assistance research team
## Tufts University

## 2018

###################################################################
# 
# Provides interfaces for aggregated movements (tasks) that perform 
# useful actions within the preset environment. These functions will 
# only perform correct tasks in the environment set up in the Tufts
# Controls, Robotics, Identification, and Signal Processing Lab at 
# 574 Boston Ave, Medford, MA

from positionControl import *
import threading
import sys
import time

def moveToDownward(lLimb, rLimb, pause_event):
	try:
		sys.path.insert(0, './mobile_commands')
	except IOError as e:
		print(e)

	curY = 	lLimb.endpoint_pose()['position'][1]
	delY = .4 - curY
	if delY > .01:
		moveOnAxis(lLimb, 'y', delY, .2, pause_event)

	if (curY <= 0.6):
		playPositionFile('./mobile_commands/default_pos_from_start.wp', lLimb, rLimb, pause_event)
	else:		
		downward = {"left_w0":0.38157772098649667,"left_w1":1.880276950750546,
					"left_w2":0.01917475984856767,"left_e0":-0.7761942786700193,
					"left_e1":0.7171360183364309,"left_s0":1.3410827038088229,
					"left_s1":-1.0565292676560787}
		lLimb.set_joint_position_speed(.5)
		lLimb.move_to_joint_positions(downward)

def openFridge(lLimb, rLimb, pause_event):
	try:
		sys.path.insert(0, './mobile_commands')
	except IOError as e:
		print(e)

	playPositionFile('./mobile_commands/open_fridge.wp', lLimb, rLimb, pause_event)


def pickBottleFromOpenFridge(lLimb,rLimb,gripper, pause_event):
	try:
		sys.path.insert(0, './mobile_commands')
	except IOError as e:
		print(e)

	playPositionFile('./mobile_commands/get_bottle_open_fridge_p1.wp', lLimb, rLimb, pause_event)
	moveOnAxis(lLimb, 'y', .08, .06, pause_event)
	time.sleep(1)
	waitForNotPause(pause_event)
	gripper.close()
	time.sleep(1)
	playPositionFile('./mobile_commands/get_bottle_open_fridge_p2.wp', lLimb, rLimb, pause_event)

def getBottleFromStart(lLimb, rLimb, gripper, pause_event):
	openFridge(lLimb, rLimb, pause_event)
	pickBottleFromOpenFridge(lLimb, rLimb, gripper, pause_event)

def moveToTableAfterRetrieve(lLimb, rLimb, gripper, pause_event):
	try:
		sys.path.insert(0, './mobile_commands')
	except IOError as e:
		print(e)

	playPositionFile('./mobile_commands/move_bottle_to_table_p1.wp', lLimb, rLimb, pause_event)
	#moveOnAxis(lLimb, 'y', -.1, .04, pause_event)
	#moveOnAxis(lLimb, 'z', -.1, .08, pause_event)
	time.sleep(1)
	waitForNotPause(pause_event)
	gripper.open()
	time.sleep(1)
	#moveOnAxis(lLimb, 'z', .08, .04, pause_event)
	moveOnAxis(lLimb, 'x', -0.5, .04, pause_event)
	playPositionFile('./mobile_commands/move_bottle_to_table_p2.wp', lLimb, rLimb, pause_event)

def getBottleFull(lLimb, rLimb, gripper, pause_event):
	openFridge(lLimb, rLimb, pause_event)
	pickBottleFromOpenFridge(lLimb, rLimb, gripper, pause_event)
	moveToTableAfterRetrieve(lLimb, rLimb, gripper, pause_event)
	closeFridge(lLimb, rLimb, pause_event)

def bottleOnTableAfterOpenFridge(lLimb, rLimb, gripper, pause_event):
	pickBottleFromOpenFridge(lLimb, rLimb, gripper, pause_event)
	moveToTableAfterRetrieve(lLimb, rLimb, gripper, pause_event)
	closeFridge(lLimb, rLimb, pause_event)

def closeFridge(lLimb, rLimb, pause_event):
	try:
		sys.path.insert(0, './mobile_commands')
	except IOError as e:
		print(e)

	#playPositionFile('./mobile_commands/close_fridge.wp', lLimb, rLimb, pause_event)
	playPositionFile('closeFridge2.wp', lLimb, rLimb, pause_event)
	moveToDownward(lLimb, rLimb, pause_event)


############### Microwave centered commands #########################

def openMicrowave(lLimb, rLimb, pause_event):
	try:
		sys.path.insert(0, './mobile_commands')
	except IOError as e:
		print(e)

	playPositionFile('./mobile_commands/openMic.wp', lLimb, rLimb, pause_event)

def closeMicrowave(lLimb, rLimb, pause_event):
	try:
		sys.path.insert(0, './mobile_commands')
	except IOError as e:
		print(e)

	playPositionFile('./mobile_commands/close_microwave.wp', lLimb, rLimb, pause_event)

def turnOnMicrowave(lLimb, rLimb, pause_event):
	try:
		sys.path.insert(0, './mobile_commands')
	except IOError as e:
		print(e)
	playPositionFile('./mobile_commands/turn_on_microwave.wp', lLimb, rLimb, pause_event)
	time.sleep(.5)
	moveOnAxis(lLimb, 'y', .07, .09, pause_event)
	time.sleep(.5)
	moveOnAxis(lLimb, 'z', -.10, .04, pause_event)	
	time.sleep(90)
	turnOffMicrowave(lLimb, rLimb, pause_event)

def turnOffMicrowave(lLimb, rLimb, pause_event):
	try:
		sys.path.insert(0, './mobile_commands')
	except IOError as e:
		print(e)

	moveOnAxis(lLimb, 'z', .15, .07, pause_event)
	moveOnAxis(lLimb, 'z', -.03, .04, pause_event)
	moveOnAxis(lLimb, 'y', -.05, .05, pause_event)
	playPositionFile('./mobile_commands/turn_off_microwave.wp', lLimb, rLimb, pause_event)

def timedMicrowave(lLimb, rLimb, pause_event, t):
	try:
		sys.path.insert(0, './mobile_commands')
	except IOError as e:
		print(e)
	playPositionFile('./mobile_commands/turn_on_microwave.wp', lLimb, rLimb, pause_event)
	time.sleep(.5)
	moveOnAxis(lLimb, 'y', .07, .09, pause_event)
	time.sleep(.5)
	moveOnAxis(lLimb, 'z', -.10, .04, pause_event)	
	time.sleep(t)
	turnOffMicrowave(lLimb, rLimb, pause_event)



def getFoodContainer(lLimb, rLimb, gripper, pause_event):
	try:
		sys.path.insert(0, './mobile_commands')
	except IOError as e:
		print(e)
	playPositionFile('./mobile_commands/get_food_from_fridge_p1.wp', lLimb, rLimb, pause_event)
	#moveOnAxis(lLimb, 'z', -.015, .03, pause_event)
	moveOnAxis(lLimb, 'y', .07, .03, pause_event)
	time.sleep(.5)
	waitForNotPause(pause_event)
	gripper.close()
	time.sleep(.5)
	moveOnAxis(lLimb, 'z', .03, .04, pause_event)
	moveOnAxis(lLimb, 'y', -.15, .03, pause_event)
	playPositionFile('./mobile_commands/get_food_from_fridge_p2.wp', lLimb, rLimb, pause_event)

def placeFoodContainerInMicrowave(lLimb, rLimb, gripper, pause_event):
	try:
		sys.path.insert(0, './mobile_commands')
	except IOError as e:
		print(e)
	#playPositionFile('./mobile_commands/put_food_in_microwave_p1.wp', lLimb, rLimb, pause_event)
	playPositionFile('putFoodInMic2.wp', lLimb, rLimb, pause_event)
	time.sleep(2)
	
	moveOnAxis(lLimb, 'y', .15, .02, pause_event)
	moveOnAxis(lLimb, 'z', -.03, .02, pause_event)
	time.sleep(1)
	#gripper.open()
	
	waitForNotPause(pause_event)
	time.sleep(1)
	gripper.open()
	time.sleep(1)
	moveOnAxis(lLimb, 'y', -.17, .02, pause_event)
	playPositionFile('./mobile_commands/put_food_in_microwave_p2.wp', lLimb, rLimb, pause_event)

def placeContainerInMicrowaveFromStart(lLimb, rLimb, gripper, pause_event):
	openMicrowave(lLimb, rLimb, pause_event)
	time.sleep(1)
	openFridge(lLimb, rLimb, pause_event)
	time.sleep(1)
	getFoodContainer(lLimb, rLimb, gripper, pause_event)
	time.sleep(1)
	placeFoodContainerInMicrowave(lLimb, rLimb, gripper, pause_event)
	closeMicrowave(lLimb, rLimb, pause_event)
	closeFridge(lLimb, rLimb, pause_event)

def placeContainerInMicrowaveFromOpenFridge(lLimb, rLimb, gripper, pause_event):
	openMicrowave(lLimb, rLimb, pause_event)
	time.sleep(1)
	getFoodContainer(lLimb, rLimb, gripper, pause_event)
	time.sleep(1)
	placeFoodContainerInMicrowave(lLimb, rLimb, gripper, pause_event)
	closeMicrowave(lLimb, rLimb, pause_event)
	closeFridge(lLimb, rLimb, pause_event)

def placeContainerInMicrowaveFromOpenMicrowave(lLimb, rLimb, gripper, pause_event):
	openFridge(lLimb, rLimb, pause_event)
	time.sleep(1)
	getFoodContainer(lLimb, rLimb, gripper, pause_event)
	time.sleep(1)
	placeFoodContainerInMicrowave(lLimb, rLimb, gripper, pause_event)
	closeMicrowave(lLimb, rLimb, pause_event)
	closeFridge(lLimb, rLimb, pause_event)

def placeContainerInMicrowaveFromOpenMicOpenFridge(lLimb, rLimb, gripper, pause_event):
	getFoodContainer(lLimb, rLimb, gripper, pause_event)
	time.sleep(1)
	placeFoodContainerInMicrowave(lLimb, rLimb, gripper, pause_event)
	closeMicrowave(lLimb, rLimb, pause_event)
	closeFridge(lLimb, rLimb, pause_event)

# uncomment these out when you record the tasks

def getFoodFromMicrowave(lLimb, rLimb, gripper, pause_event):
	gripper.open()
	openMicrowave(lLimb, rLimb, pause_event)
	time.sleep(1)
	playPositionFile('./mobile_commands/get_food_from_microwave_p1.wp', lLimb, rLimb, pause_event)
	time.sleep(1)
	moveOnAxis(lLimb, 'z', -.02, .02, pause_event)
	moveOnAxis(lLimb, 'y', .17, .02, pause_event)
	waitForNotPause(pause_event)
	time.sleep(1)
	gripper.close()
	time.sleep(1)
	moveOnAxis(lLimb, 'z', .02, .02, pause_event)
	#moveOnAxis(lLimb, 'y', -.17, .02, pause_event)
	playPositionFile('./mobile_commands/get_food_from_microwave_p2.wp', lLimb, rLimb, pause_event)
	gripper.open()
	moveOnAxis(lLimb, 'y', -.06, .02, pause_event)
	playPositionFile('./mobile_commands/get_food_from_microwave_p3.wp', lLimb, rLimb, pause_event)
	closeMicrowave(lLimb, rLimb, pause_event)
	moveToDownward(lLimb, rLimb, pause_event)

def getFoodFromOpenMicrowave(lLimb, rLimb, gripper, pause_event):
	gripper.open()
	time.sleep(1)
	#playPositionFile('./mobile_commands/get_food_from_microwave_p1.wp', lLimb, rLimb, pause_event)
	playPositionFile('putFoodInMic.wp', lLimb, rLimb, pause_event)
	time.sleep(1)
	moveOnAxis(lLimb, 'z', -.02, .02, pause_event)
	moveOnAxis(lLimb, 'y', .17, .02, pause_event)
	waitForNotPause(pause_event)
	time.sleep(1)
	gripper.close()
	time.sleep(1)
	moveOnAxis(lLimb, 'z', .02, .02, pause_event)
	#moveOnAxis(lLimb, 'y', -.17, .02, pause_event)
	playPositionFile('./mobile_commands/get_food_from_microwave_p2.wp', lLimb, rLimb, pause_event)
	gripper.open()
	moveOnAxis(lLimb, 'y', -.06, .02, pause_event)
	playPositionFile('./mobile_commands/get_food_from_microwave_p3.wp', lLimb, rLimb, pause_event)
	closeMicrowave(lLimb, rLimb, pause_event)
	moveToDownward(lLimb, rLimb, pause_event)


############# new unintegrated commands ############

def tablePickUpMode(lLimb, rLimb, gripper, pause_event):
	playPositionFile('./mobile_commands/table_grab_mode.wp', lLimb, rLimb, pause_event)

def groundPickUpMode(lLimb, rLimb, gripper, pause_event):
	playPositionFile('./mobile_commands/ground_grab_mode.wp', lLimb, rLimb, pause_event)

def pickFromFloor(lLimb, rLimb, gripper, pause_event):
	gripper.open()
	playPositionFile('./mobile_commands/pick3.wp', lLimb, rLimb, pause_event)
	gripper.close()
	playPositionFile('./mobile_commands/pick2.wp', lLimb, rLimb, pause_event)
	#playPositionFile('./mobile_commands/pickFromFloor2.wp', lLimb, rLimb, pause_event)
	#playPositionFile('./mobile_commands/pickFromFloor3.wp', lLimb, rLimb, pause_event)


def dropObject(lLimb, rLimb, gripper, pause_event):
	gripper.open()
	#playPositionFile('./mobile_commands/dropObject.wp', lLimb, rLimb, pause_event)
	#gripper.close()


def putOnTable(lLimb, rLimb, gripper, pause_event):
	playPositionFile('./mobile_commands/putOnTable1.wp', lLimb, rLimb, pause_event)
	gripper.open()
	playPositionFile('./mobile_commands/putOnTable2.wp', lLimb, rLimb, pause_event)
	gripper.close()
# def autoPark()
	# put arms into specific position 
	# auto park
	# move to downward

'''
def pickUpBottleFromTable(lLimb, rLimb, gripper, pause_event):
	playPositionFile('pickUpBottleFromTable.wp', lLimb, rLimb, pause_event)
	quatOrient = euler2Quat(0, 90, 180)
	setOrientation(lLimb, quatOrient['qx'], quatOrient['qy'], quatOrient['qz'], quatOrient['qw'])
	moveOnAxis(lLimb, 'z', -.17, .05, pause_event)
	waitForNotPause(pause_event)
	time.sleep(.5)
	gripper.close()
	moveOnAxis(lLimb, 'z', .15, .1, pause_event)
	playPositionFile('pickUpBottleFromTableP2.wp', lLimb, rLimb, pause_event)

def putBottleInHolder(lLimb, rLimb, gripper, pause_event):
	playPositionFile('putBottleInHolderP1.wp', lLimb, rLimb, pause_event)
	# moveOnAxis(lLimb, 'x', -.02, .02, pause_event)
	# moveOnAxis(lLimb, 'y', -.03, .02, pause_event)
	# moveOnAxis(lLimb, 'z', -.15, .03, pause_event)
	# time.sleep(1)
	# gripper.open()
	# moveOnAxis(lLimb, 'z', .15, .07, pause_event)
'''

#### TESTING FUNCTION #####################
def tester(lLimb, rLimb, gripper, pause_event):
	# pickFromFloor(lLimb, rLimb, gripper, pause_event)
	#SdropObject(lLimb, rLimb, gripper, pause_event)
	# putOnTable(lLimb, rLimb, gripper, pause_event)
	#moveOnAxis(limb, 'x', -4, .03)
	playPositionFile('./mobile_commands/put_food_in_microwave_p1.wp', lLimb, rLimb, pause_event)
