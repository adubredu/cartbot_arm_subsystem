## Tasks of aggregated movement


from positionControlPackage import *

import sys

def moveLeftToDatum(lLimb, rLimb):
	try:
		sys.path.insert(0, './ProductFiles20180201')
	except IOError as e:
		print(e)

	curY = 	lLimb.endpoint_pose()['position'][1]
	delY = .8 - curY
	moveOnAxis(lLimb, 'y', delY, .2)

	downWard = {'left_w0': -3.049937301513174, 'left_w1': -1.3575729972785913, 
	 			'left_w2': -0.13192234775814557, 'left_e0': -0.616276781532965, 
	 			'left_e1': 1.5899710866432313, 'left_s0': 1.2237331735355887, 'left_s1': -1.3936215457938983}
	lLimb.set_joint_position_speed(.5)
	lLimb.move_to_joint_positions(downWard)

	playPositionFile('leftDatum.wp', lLimb, rLimb)

def openFridge(lLimb, rLimb):
	try:
		sys.path.insert(0, './ProductFiles20180213')
	except IOError as e:
		print(e)

	moveLeftToDatum(lLimb,rLimb)

	playPositionFile('openFridgeP1.wp', lLimb, rLimb)
	moveOnAxis(lLimb, 'z', .1, .04)
	moveOnAxis(lLimb, 'y', -.10, .03)
	moveOnAxis(lLimb, 'x', -.2, .03)
	playPositionFile('openFridgeP2.wp', lLimb, rLimb)
	moveOnAxis(lLimb, 'x', .08, .04)
	playPositionFile('openFridgeP3.wp', lLimb, rLimb)

def pickUpWaterBottle(lLimb, rLimb):
	moveOnAxis(lLimb, 'z', .2, .04)

def tester(limb):
	orient = limb.endpoint_pose()['orientation']
	# moveOnAxis(limb, 'x', -4, .03)