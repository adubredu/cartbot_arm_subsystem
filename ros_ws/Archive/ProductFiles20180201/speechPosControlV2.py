import argparse
import sys
import string

import rospy

from std_msgs.msg import (
    UInt16,
)

import baxter_interface as baxter

import speech_recognition as SR

from positionControlSandbox import *

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

rospy.init_node('speechControl')
baxter_enabler = baxter.RobotEnable(versioned=True)
baxter_enabler.enable()

sens = 1

commands = [("forward", sens), ("backward", sens), ("left", sens), 
            ("right", sens), ("higher", sens), ("lower", sens), 
            ("close", sens), ("open", sens), ("quit", sens)
            ]

limb = baxter.Limb('left')
gripper = baxter.Gripper('left')

if not gripper.calibrate():
    print("gripper did not calibrate")
    sys.exit()

curX = 0.0
curY = 0.98
curZ = 0.4

orient = 

jointPos = xyzToAngles(curX, curY, curZ)
limb.move_to_joint_positions(jointPos)


Del = .1

r = SR.Recognizer()
r.operation_timeout = 5

while True:
    valComm = False
    with SR.Microphone() as source:
        print("forward, backward, left, right, higher, lower, close, open")
        r.adjust_for_ambient_noise(source, 2)
        audio = r.listen(source, 10, 5)

    with open("gspeechcreds.json", 'r') as credFile:
        credentials = credFile.read()


    GOOGLE_CLOUD_SPEECH_CREDENTIALS = credentials
    try:
        command = r.recognize_sphinx(audio_data=audio, language='en-US', keyword_entries=commands)
        valComm = True
    except SR.UnknownValueError:
        print("Sphinx could not understand audio")
    except SR.RequestError as e:
        print("Sphinx error; {0}".format(e))

    if valComm:
        command = str(command)

        command = command.strip()
        command = command.replace(" ", '')
    
        print(command)
        
        if 'forward' in command:
            curY = clamp(curY + Del, .98, 1.5)
        if 'backward' in command:
            curY = clamp(curY - Del, .98, 1.5)
        if 'left' in command:
            curX = curX - Del
        if 'right' in command:
            curX = curX + Del
        if 'higher' in command:
            curZ = curZ + Del
        if 'lower' in command:
            curZ = curZ - Del
        if 'close' in command:
            gripper.close()
        if 'open' in command:
            gripper.open()
        if 'quit' in command:
            print("here")
            raise SystemExit

    print("curX: {}, curY: {}, curZ: {}".format(curX, curY, curZ))

    jointPos = xyzToAngles(curX, curY, curZ)
    

    limb.move_to_joint_positions(jointPos)