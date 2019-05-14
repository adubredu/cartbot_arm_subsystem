import rospy as ros
import baxter_interface as baxter
import json

ros.init_node('cartRecorder')

lLimb = baxter.Limb('left')

fName = '{}.wp'.format(raw_input('Type file name: '))
file = open(fName, 'w+')

wpoints = []
done = False
while not done:
	record = raw_input('Press enter to record Current Position, enter anything else to quit: ')
	if record != "": 
		done = True
	else:
		pose = lLimb.joint_angles()
		wpoints.append(pose)

jsonPose = json.dumps(wpoints, separators=(',',':'))
file.write(jsonPose)
file.close()