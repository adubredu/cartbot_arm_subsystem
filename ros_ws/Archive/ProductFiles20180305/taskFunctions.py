## Tasks of aggregated movement


from positionControl import *

import sys

def moveToDownward(lLimb, rLimb):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	curY = 	lLimb.endpoint_pose()['position'][1]
	delY = .8 - curY
	moveOnAxis(lLimb, 'y', delY, .2)

	downward = {'left_w0': -3.049937301513174, 'left_w1': -1.3575729972785913, 
	 			'left_w2': -0.13192234775814557, 'left_e0': -0.616276781532965, 
	 			'left_e1': 1.5899710866432313, 'left_s0': 1.2237331735355887, 'left_s1': -1.3936215457938983}
	lLimb.set_joint_position_speed(.5)
	lLimb.move_to_joint_positions(downward)

def openFridge(lLimb, rLimb):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	moveToDownward(lLimb,rLimb)

	playPositionFile('openFridgeP1.wp', lLimb, rLimb)

def highFive(lLimb, rLimb):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	moveToDownward(lLimb,rLimb)

	playPositionFile('highFive.wp', lLimb, rLimb)

def pickBottleFromOpenFridge(lLimb,rLimb,gripper):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	playPositionFile('pickUpBottleFromOpenFridgeP1.wp', lLimb, rLimb)
	moveOnAxis(lLimb, 'y', .06, .06)
	time.sleep(1)
	gripper.close()
	time.sleep(1)
	playPositionFile('pickUpBottleFromOpenFridgeP2.wp', lLimb, rLimb)

def getBottleFromStart(lLimb, rLimb, gripper):
	openFridge(lLimb, rLimb)
	pickBottleFromOpenFridge(lLimb, rLimb, gripper)

def moveToTableAfterRetrieve(lLimb, rLimb, gripper):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	playPositionFile('moveToTableAfterRetrieve.wp', lLimb, rLimb)
	moveOnAxis(lLimb, 'z', -.18, .08)
	time.sleep(1)
	gripper.open()
	time.sleep(1)
	moveOnAxis(lLimb, 'z', .08, .04)
	moveOnAxis(lLimb, 'x', -.06, .06)
	playPositionFile('moveToTableAfterRetrieveP2.wp', lLimb, rLimb)

def getBottleFull(lLimb, rLimb, gripper):
	openFridge(lLimb, rLimb)
	pickBottleFromOpenFridge(lLimb, rLimb, gripper)
	moveToTableAfterRetrieve(lLimb, rLimb, gripper)
	closeFridge(lLimb, rLimb)

def bottleOnTableAfterOpenFridge(lLimb, rLimb, gripper):
	pickBottleFromOpenFridge(lLimb, rLimb, gripper)
	moveToTableAfterRetrieve(lLimb, rLimb, gripper)
	closeFridge(lLimb, rLimb)

def closeFridge(lLimb, rLimb):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	playPositionFile('closeFridge.wp', lLimb, rLimb)
	moveToDownward(lLimb, rLimb)

def openTheMicrowave(lLimb, rLimb):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	playPositionFile('openMicrowaveP1.wp', lLimb, rLimb)

def closeTheMicrowave(lLimb, rLimb):
	try:
		sys.path.insert(0, './ProductFiles')
	except IOError as e:
		print(e)

	playPositionFile('closeMicrowaveP1.wp', lLimb, rLimb)

def turnOnMicrowave(lLimb, rLimb):
	playPositionFile('turnOnMicrowave.wp', lLimb, rLimb)
	moveOnAxis(lLimb, 'z', -.09, .08)
	playPositionFile('turnOnMicrowaveP2.wp', lLimb, rLimb)

def turnOffMicrowave(lLimb, rLimb):
	moveOnAxis(lLimb, 'z', .09, .07)
	moveOnAxis(lLimb, 'y', -.05, .05)
	playPositionFile('turnOffMicrowave.wp', lLimb, rLimb)


def tester(lLimb, rLimb, gripper):
	playPositionFile('test1002.wp', lLimb, rLimb)
	# moveOnAxis(limb, 'x', -4, .03)