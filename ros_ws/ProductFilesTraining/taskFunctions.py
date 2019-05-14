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

def moveToDownward(lLimb, rLimb, pause_event):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	curY = 	lLimb.endpoint_pose()['position'][1]
	delY = .4 - curY
	if delY > .01:
		moveOnAxis(lLimb, 'y', delY, .2, pause_event)

	downward = {'left_w0': -3.049937301513174, 'left_w1': -1.3575729972785913, 
	 			'left_w2': -0.13192234775814557, 'left_e0': -0.616276781532965, 
	 			'left_e1': 1.5899710866432313, 'left_s0': 1.2237331735355887, 'left_s1': -1.3936215457938983}
	lLimb.set_joint_position_speed(.5)
	lLimb.move_to_joint_positions(downward)

def openFridge(lLimb, rLimb, pause_event):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	playPositionFile('openFridgeP1.wp', lLimb, rLimb, pause_event)

def highFive(lLimb, rLimb, pause_event):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	moveToDownward(lLimb,rLimb, pause_event)

	playPositionFile('highFive.wp', lLimb, rLimb, pause_event)

def pickBottleFromOpenFridge(lLimb,rLimb,gripper, pause_event):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	playPositionFile('pickUpBottleFromOpenFridgeP1.wp', lLimb, rLimb, pause_event)
	moveOnAxis(lLimb, 'y', .08, .06, pause_event)
	time.sleep(1)
	waitForNotPause(pause_event)
	gripper.close()
	time.sleep(1)
	playPositionFile('pickUpBottleFromOpenFridgeP2.wp', lLimb, rLimb, pause_event)

def getBottleFromStart(lLimb, rLimb, gripper, pause_event):
	openFridge(lLimb, rLimb, pause_event)
	pickBottleFromOpenFridge(lLimb, rLimb, gripper, pause_event)

def moveToTableAfterRetrieve(lLimb, rLimb, gripper, pause_event):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	playPositionFile('moveToTableAfterRetrieve.wp', lLimb, rLimb, pause_event)
	moveOnAxis(lLimb, 'y', -.1, .04, pause_event)
	moveOnAxis(lLimb, 'z', -.1, .08, pause_event)
	time.sleep(1)
	waitForNotPause(pause_event)
	gripper.open()
	time.sleep(1)
	moveOnAxis(lLimb, 'z', .08, .04, pause_event)
	moveOnAxis(lLimb, 'x', -.1, .06, pause_event)
	playPositionFile('moveToTableAfterRetrieveP2.wp', lLimb, rLimb, pause_event)

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
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	playPositionFile('closeFridge.wp', lLimb, rLimb, pause_event)
	moveToDownward(lLimb, rLimb, pause_event)

def openMicrowave(lLimb, rLimb, pause_event):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	playPositionFile('openMicrowave.wp', lLimb, rLimb, pause_event)

def closeMicrowave(lLimb, rLimb, pause_event):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	playPositionFile('closeMicrowave.wp', lLimb, rLimb, pause_event)

def turnOnMicrowave(lLimb, rLimb, pause_event):
	playPositionFile('turnOnMicrowave.wp', lLimb, rLimb, pause_event)
	time.sleep(.5)
	moveOnAxis(lLimb, 'y', .07, .09, pause_event)
	time.sleep(.5)
	moveOnAxis(lLimb, 'z', -.10, .04, pause_event)	
	time.sleep(90)
	turnOffMicrowave(lLimb, rLimb, pause_event)

def turnOffMicrowave(lLimb, rLimb, pause_event):
	moveOnAxis(lLimb, 'z', .15, .07, pause_event)
	moveOnAxis(lLimb, 'z', -.03, .04, pause_event)
	moveOnAxis(lLimb, 'y', -.05, .05, pause_event)
	playPositionFile('turnOffMicrowave.wp', lLimb, rLimb, pause_event)

def timedMicrowave(lLimb, rLimb, pause_event, t):
	playPositionFile('turnOnMicrowave.wp', lLimb, rLimb, pause_event)
	time.sleep(.5)
	moveOnAxis(lLimb, 'y', .07, .09, pause_event)
	time.sleep(.5)
	moveOnAxis(lLimb, 'z', -.10, .04, pause_event)	
	time.sleep(t)
	turnOffMicrowave(lLimb, rLimb, pause_event)

def getFoodContainer(lLimb, rLimb, gripper, pause_event):
	playPositionFile('getFoodContainer.wp', lLimb, rLimb, pause_event)
	#moveOnAxis(lLimb, 'z', -.015, .03, pause_event)
	moveOnAxis(lLimb, 'y', .07, .03, pause_event)
	time.sleep(.5)
	waitForNotPause(pause_event)
	gripper.close()
	time.sleep(.5)
	moveOnAxis(lLimb, 'z', .03, .04, pause_event)
	moveOnAxis(lLimb, 'y', -.15, .03, pause_event)
	playPositionFile('getFoodContainerP2.wp', lLimb, rLimb, pause_event)

def placeFoodContainerInMicrowave(lLimb, rLimb, gripper, pause_event):
	playPositionFile('placeFoodInMicrowave.wp', lLimb, rLimb, pause_event)
	time.sleep(1)
	moveOnAxis(lLimb, 'y', .17, .02, pause_event)
	moveOnAxis(lLimb, 'z', -.02, .02, pause_event)
	waitForNotPause(pause_event)
	time.sleep(1)
	gripper.open()
	time.sleep(1)
	moveOnAxis(lLimb, 'y', -.17, .02, pause_event)
	playPositionFile('putFoodInMicrowaveP2.wp', lLimb, rLimb, pause_event)

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

def getFoodFromMicrowave(lLimb, rLimb, gripper, pause_event):
	gripper.open()
	openMicrowave(lLimb, rLimb, pause_event)
	time.sleep(1)
	playPositionFile('getFoodFromMicrowaveP1.wp', lLimb, rLimb, pause_event)
	time.sleep(1)
	moveOnAxis(lLimb, 'z', -.02, .02, pause_event)
	moveOnAxis(lLimb, 'y', .17, .02, pause_event)
	waitForNotPause(pause_event)
	time.sleep(1)
	gripper.close()
	time.sleep(1)
	moveOnAxis(lLimb, 'z', .02, .02, pause_event)
	moveOnAxis(lLimb, 'y', -.17, .02, pause_event)
	playPositionFile('getFoodFromMicrowaveP2.wp', lLimb, rLimb, pause_event)
	gripper.open()
	playPositionFile('getFoodFromMicrowaveP3.wp', lLimb, rLimb, pause_event)
	moveOnAxis(lLimb, 'x', -.15, .05, pause_event)
	playPositionFile('getFoodFromMicrowaveP4.wp', lLimb, rLimb, pause_event)
	closeMicrowave(lLimb, rLimb, pause_event)
	moveToDownward(lLimb, rLimb, pause_event)

def getFoodFromOpenMicrowave(lLimb, rLimb, gripper, pause_event):
	gripper.open()
	time.sleep(1)
	playPositionFile('getFoodFromMicrowaveP1.wp', lLimb, rLimb, pause_event)
	time.sleep(1)
	moveOnAxis(lLimb, 'z', -.02, .02, pause_event)
	moveOnAxis(lLimb, 'y', .17, .02, pause_event)
	waitForNotPause(pause_event)
	time.sleep(1)
	gripper.close()
	time.sleep(1)
	moveOnAxis(lLimb, 'z', .02, .02, pause_event)
	moveOnAxis(lLimb, 'y', -.17, .02, pause_event)
	playPositionFile('getFoodFromMicrowaveP2.wp', lLimb, rLimb, pause_event)
	gripper.open()
	playPositionFile('getFoodFromMicrowaveP3.wp', lLimb, rLimb, pause_event)
	moveOnAxis(lLimb, 'x', -.15, .05, pause_event)
	playPositionFile('getFoodFromMicrowaveP4.wp', lLimb, rLimb, pause_event)
	closeMicrowave(lLimb, rLimb, pause_event)
	moveToDownward(lLimb, rLimb, pause_event)

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


def tester(lLimb, rLimb, gripper):
	playPositionFile('test1002.wp', lLimb, rLimb, pause_event)
	# moveOnAxis(limb, 'x', -4, .03)

def MoveItem(lLimb, rLimb, gripper, pause_event):
	playPositionFile('MoveArmTowardsItem1.wp', lLimb, rLimb, pause_event)
	time.sleep(.5)
	moveOnAxis(lLimb, 'z', -.25, .05, pause_event)
	time.sleep(.5)
	gripper.close()
	time.sleep(.5)
	moveOnAxis(lLimb, 'z', .2, .05, pause_event)
	playPositionFile('MoveArmBackItem1.wp', lLimb, rLimb, pause_event)
	moveOnAxis(lLimb, 'z', -.15, .05, pause_event)
	gripper.open()
	time.sleep(.5)
	moveOnAxis(lLimb, 'z', .25, .05, pause_event)
	moveOnAxis(lLimb, 'y', -.3, .1, pause_event)
	moveToDownward(lLimb, rLimb, pause_event)

def CloseFridgeMoveItem(lLimb, rLimb, gripper, pause_event):
	closeFridge(lLimb, rLimb, pause_event)
	time.sleep(.5)
	MoveItem(lLimb, rLimb, gripper, pause_event)
	


