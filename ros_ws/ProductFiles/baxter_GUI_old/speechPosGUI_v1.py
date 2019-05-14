import argparse
import sys
import string
import time
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
import pyttsx
import speech_recognition as sr
from positionControl import *
from taskFunctions import *
from runTasks import *
from std_msgs.msg import String
from baxter_core_msgs.msg import CollisionDetectionState
from sensor_msgs.msg import Image
import time
import cv2
import cv_bridge
from Tkinter import *

#fix the mic bug! go back into archives
#TODO - add self.commands one by one

class Application(Frame):

    #Global Variables#
    rawCommand = ""
    collisionState = False
    baxter_enabler = None
    collisionSubs = None

    #baxter globals
    lLimb = None
    rLimb = None
    lGripper = None
    rGripper = None

    # Text to speech engine g;obals
    t2s = pyttsx.init()
    voices = t2s.getProperty('voices')
    t2s.setProperty('voice', 'english')
    t2s.setProperty('rate', 150)

    #mic globals
    r = None
    m = None

    # Environment Tracking Booleans
    env = ({'fridgeOpen': False, 'hasBottle': False, 'bottleOnTable':False, 'microwaveOpen': False, 'holdingSomething': False, 
           'microwaveOn': False, 'foodInMicrowave': False})

    slow = .02 # m/s
    fast = .2 # m/s

    command = ""
    newCommand = ""
    lastCommand = ""
    pause_event = TEvent()

    def createWidgets(self):
        #Dialog title
        self.dialog_label = Label(self, text="Dialog", fg="black", bg="light blue", 
                                    width=60, height=2, font=("Ariel", 20))
        self.dialog_label.pack()
        #Dialog box
        self.dialog_box = Label(self, bg="white", padx=10, pady=10, 
                                    borderwidth=1, relief=SOLID, anchor="nw",
                                    width=100, height=4, font=("Ariel", 15))
        self.dialog_content = StringVar()
        self.dialog_content.set("Listening...")
        self.dialog_box["textvariable"] = self.dialog_content
        self.dialog_box.pack(fill = X)
        #command label
        self.command_box = Label(self, fg="black", bg="lightgrey",
                                 bd=1, relief=SOLID, width=60, height=2, 
                                 font=("Ariel", 20))
        self.curr_command = StringVar()
        self.curr_command.set("Waiting...")
        self.command_box["textvariable"] = self.curr_command
        self.command_box.pack()
        #blank label separator
        self.blank = Label(self, width=60, height=2)
        self.blank.pack()
        #environment title
        self.env_label = Label(self, text="Environment", fg="black", bg="light blue", 
                                    width=60, height=2, font=("Ariel", 20))
        self.env_label.pack()        
        #environment tokens
        self.env1 = Label(self, text="ENV-1", bg="dark grey",
                            bd=1, relief=SOLID, width=60, height=5,
                            font=("Ariel", 20))
        self.env1.pack()
        self.env2 = Label(self, text="ENV-2", bg="dark grey",
                            bd=1, relief=SOLID, width=60, height=5,
                            font=("Ariel", 20))
        self.env2.pack()

        self.env3 = Label(self, text="ENV-3", bg="dark grey",
                            bd=1, relief=SOLID, width=60, height=5,
                            font=("Ariel", 20))
        self.env3.pack()

        self.env4 = Label(self, text="ENV-4", bg="dark grey",
                            bd=1, relief=SOLID, width=60, height=5,
                            font=("Ariel", 20))
        self.env4.pack()
    #end of create widgets

    def update_command_box(self, command):
        self.curr_command.set(command)
        self.command_box["bg"] = "green"

    # clamping function to constrain arm movement
    #move to helpers?
    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    # callback function when audio data is obtained
    def heard(self, recognizer, audio):
        # Defining self.Commands to be accepted

        if self.lastAliveStatus and not self.task.is_alive():
            self.t2s.say("  ")
            self.t2s.say("  ")
            self.t2s.say("  ")
            self.t2s.say("  ")       # clear text to speech queue
            self.t2s.say(" Done with " + self.lastAliveName)
            self.t2s.runAndWait()
            self.lastAliveStatus = False
            self.curr_command.set("Waiting..")
            self.command_box["bg"] = "light grey"


        credsJson = ""
        with open('baxter-helper-bot-gspeechcreds.json', 'r') as gspeechcreds:
            credsJson = gspeechcreds.read()

        print("in heard")
        
        sens = 1
        commands = ["move", "arm", "forward", "backward", "left", "right", "up", "down", 
                    "higher", "lower", "close", "hand", "open", "stop", "stop", "stop", 
                    "faster", "slower", "fridge", "zero", "get", "water bottle", "fridge", 
                    "place on", "table", "fridge is open", "holding something", "fridge is closed", 
                    "hand is empty", "microwave", "start", "turn off", "continue", "cook",
                    "put", "food", "is open", "get the food"]
        print('trying to recognize')
        try:
            commandIter = [command[0] for command in commands]
            self.rawCommand = recognizer.recognize_google_cloud(audio_data=audio, language='en-US', 
                                                           credentials_json=credsJson, preferred_phrases=commands)

            self.dialog_content.set(self.rawCommand)
            print(self.rawCommand)
            self.run_commands()
        except sr.UnknownValueError:
            #if self.rawCommand == "":
                print("listening")
                self.dialog_content.set("Listening...")
                pass
            #print("could not understand audio")
          #  pass
        except sr.RequestError as e:
            print("Recognition error; {}".format(e))

    #move to helpers?
    def collisionDetection(self, data):
        collisionState = data.collision_state
        if collisionState:
            self.rawCommand = ""

    def terminate_thread(self, thread):
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

    #baxter setup
    def setup_baxter(self):
        print "setting up baxter"
        rospy.init_node('speechControl')
        self.lLimb = baxter.Limb('left')
        self.rLimb = baxter.Limb('right')
        self.lGripper = baxter.Gripper('left')
        self.rGripper = baxter.Gripper('right')
        self.baxter_enabler = baxter.RobotEnable(versioned=True)
        self.baxter_enabler.enable()
        # Set up subscriber for collision detection
        self.collisionSubs = rospy.Subscriber(name='/robot/limb/left/collision_detection_state', 
                                     data_class=CollisionDetectionState, callback=self.collisionDetection, 
                                     buff_size=100)
        if not self.lGripper.calibrate():
            print("left gripper did not calibrate")
            sys.exit()

        # amp up gripper holding force
        self.lGripper.set_holding_force(100)
        self.lGripper.set_moving_force(100)
        self.rGripper.set_holding_force(100)
        self.rGripper.set_moving_force(100)
        self.lLimb.set_joint_position_speed(.5)
        self.lLimb.set_command_timeout(2)
        strtPose = self.lLimb.endpoint_pose()
    #end setup baxter

    #speech recognition setup
    def setup_speech_recognition(self):
        self.r = sr.Recognizer()
        self.m = sr.Microphone()
        self.r.pause_threshold = .5
        self.r.dynamic_energy_threshold = False
        #r.dynamic_energy_adjustment_damping = 0.5
        #r.dynamic_energy_adjustment_ratio = 2
        with self.m as source:
            self.r.adjust_for_ambient_noise(source, 1)
    #end speech recognition setup

    def get_command(self):
        self.move_to_zero()
        self.lastAliveStatus = False
        self.lastAliveName = ""
        self.stopListening = self.r.listen_in_background(self.m, self.heard, phrase_time_limit=4)
        print("raw self.command is", self.rawCommand)
        #print("in get self.command, calling run")
        #self.run_commands()

        


    def move_to_zero(self):
        #Move to Zero position
        self.task = Thread(target=moveToDownward, args=(self.lLimb, self.rLimb, self.pause_event), name="movingToZero")
        print(self.task.name)
        self.task.daemon = True
        self.task.start()
        self.lastAliveStatus = True


    def run_commands(self):
        # Check if self.task has ended
        print "in run self.command"

        self.newCommand = self.rawCommand
        # Check if self.command has changed
        if self.command != self.newCommand:
            if self.newCommand != "": 
                self.speed = self.slow
            self.command = self.newCommand
            orient = self.lLimb.endpoint_pose()['orientation']
            print("last: {}, this: {}".format(self.lastCommand, self.command))

        if self.collisionState:
            print "Can't Move Here"
            if self.task and self.task.is_alive():
                self.rawCommand = "stop"

        #### Directional self.Commands #####
        if 'move arm forward' in self.command:
            self.terminate_thread(self.task)
            self.task = Thread(target=moveOnAxis, args=(self.lLimb, 'y', 4, self.speed, self.pause_event), name="movingForward")
            print(self.task.name)
            self.pause_event.clear()
            self.task.daemon = True
            self.task.start()
            self.rawCommand = ""
        if 'move arm backward' in self.command:
            self.terminate_thread(self.task)
            self.task = Thread(target=moveOnAxis, args=(self.lLimb, 'y', -4, self.speed, self.pause_event), name="movingBackward")
            print(self.task.name)
            self.pause_event.clear()
            self.task.daemon = True
            self.task.start()
            self.rawCommand = ""
        if 'move arm left' in self.command:
            self.terminate_thread(self.task)
            self.task = Thread(target=moveOnAxis, args=(self.lLimb, 'x', -4, self.speed, self.pause_event), name="movingLeft")
            print(self.task.name)
            self.pause_event.clear()
            self.task.daemon = True
            self.task.start()
            self.rawCommand = ""
        if 'move arm right' in self.command:
            self.terminate_thread(self.task)
            self.task = Thread(target=moveOnAxis, args=(self.lLimb, 'x', 4, self.speed, self.pause_event), name="movingRight")
            print(self.task.name)
            self.pause_event.clear()
            self.task.daemon = True
            self.task.start()
            self.rawCommand = ""
        if  'arm' in self.command and ('higher' in self.command or 'up' in self.command):
            self.terminate_thread(self.task)
            self.task = Thread(target = moveOnAxis, args=(self.lLimb, 'z', 4, self.speed, self.pause_event), name="movingHigher")
            print(self.task.name)
            self.pause_event.clear()
            self.task.daemon = True
            self.task.start()
            self.rawCommand = ""
        if  'arm' in self.command and ('lower' in self.command or 'down' in self.command):
            self.terminate_thread(self.task)
            self.task = Thread(target = moveOnAxis, args=(self.lLimb, 'z', -4, self.speed, self.pause_event), name="movingLower")
            print(self.task.name)
            self.pause_event.clear()
            self.task.daemon = True
            self.task.start()
            self.rawCommand = ""

        ### End Directional self.Commands ###

        ### Gripper Direct self.Commands ###
        if 'close' in self.command and ('hand' in self.command or 'gripper' in self.command):
            self.terminate_thread(self.task)
            self.lGripper.close()
            self.rawCommand = ""
        if 'open' in self.command and ('hand' in self.command or 'gripper' in self.command):
            self.terminate_thread(self.task)
            self.lGripper.open()
            self.rawCommand = ""
        ### End Gripper Direct self.Commands ###

        ### Stop and continue self.Commands ###
        if 'stop' in self.command:
            self.rawCommand = ""
            self.pause_event.set()
        if 'continue' in self.command:
            self.rawCommand = ""
            self.pause_event.clear()

        ### End Stop and continue self.commands ###

        ### Begin self.speed self.commands ###
        if 'go faster' in self.command:
            self.terminate_thread(self.task)
            self.speed = fast
            self.command = lastCommand
            self.rawCommand = lastCommand
        if 'go slower' in self.command:
            self.terminate_thread(self.task)
            self.speed = slow
            self.command = lastCommand
            self.rawCommand = lastCommand
        ### End self.speed self.commands ###

        # Begin fridge commands # 
        if 'open' in self.command and 'the fridge' in self.command:
            self.terminate_thread(self.task)
            #if not env['fridgeOpen']:
            if not False:
                self.task = Thread(target=openFridge, args=(self.lLimb, self.rLimb, self.pause_event), name="openingFridge")
                self.pause_event.clear()
                self.task.daemon = True
                self.task.start()
                print(self.task.name)
                self.rawCommand = ""
                self.update_command_box("Opening Fridge")
        '''        
        if 'get' in self.command and 'water bottle' in self.command:
            terminate_thread(self.task)
            if not env['fridgeOpen'] and not env['hasBottle']:
                task = Thread(target=getBottleFromStart, args=(lLimb, rLimb,lGripper, pause_event), name="gettingBottleFromStart")
                pause_event.clear()
                task.daemon = True
                task.start()
                print(task.name)
            if env['fridgeOpen'] and not env['hasBottle']:
                task = Thread(target=pickBottleFromOpenFridge, args=(lLimb, rLimb,lGripper, pause_event), name="gettingBottleFromOpenFridge")
                pause_event.clear()
                task.daemon = True
                task.start()
                print(task.name)
            if env['fridgeOpen'] and env['hasBottle']:
                pass
            rawCommand = ""
        if 'close' in command and 'the fridge' in command:
            terminate_thread(task)
            if env['fridgeOpen'] and not env['holdingSomething']:
                task = Thread(target=closeFridge, args=(lLimb, rLimb, pause_event), name="closingFridge")
                pause_event.clear()
                task.daemon = True
                task.start()
                print(task.name)
            rawCommand = ""
        if 'place' in command and 'table' in command:
            terminate_thread(task)
            if env['hasBottle']:
                task = Thread(target=moveToTableAfterRetrieve, args=(lLimb, rLimb,lGripper, pause_event), name="placingOnTable")
                pause_event.clear()
                task.daemon = True
                task.start()
                print(task.name)
            else:
                print("I don't have the bottle right now")
                print(task.name)
            rawCommand = ""
        if 'put' in command and 'water bottle' in command and 'table' in command:
            terminate_thread(task)
            if not env['fridgeOpen'] and not env['hasBottle']:
                task = Thread(target=getBottleFull, args=(lLimb, rLimb,lGripper,pause_event), name="gettingBottlePlacingOnTable")
                pause_event.clear()
                task.daemon = True
                task.start()
                print(task.name)
            elif env['fridgeOpen'] and not env['hasBottle']:
                task = Thread(target=bottleOnTableAfterOpenFridge, args=(lLimb, rLimb, lGripper, pause_event), name="gettingBottleFromOpenFridgeAndPlacingOnTable")
                pause_event.clear()
                task.daemon = True
                task.start()
                print(task.name)
            elif env['fridgeOpen'] and env['hasBottle']:
                task = Thread(target=moveToTableAfterRetrieve, args=(lLimb, rLimb,lGripper, pause_event), name="placingOnTable")
                pause_event.clear()
                task.daemon = True
                task.start()
                print(task.name)
            rawCommand = ""
        '''
        # End Fridge task commands #


        ### miscellaneous commands ###
        if 'move to zero' in self.command:
            '''
            self.terminate_thread(self.task)
            self.task = Thread(target=moveToDownward, args=(self.lLimb, self.rLimb, self.pause_event), name="movingToZero")
            print(self.task.name)
            self.pause_event.clear()
            self.task.daemon = True
            self.task.start()
            self.update_command_box("Moving to zero")
            self.rawCommand = ""
            '''
            runMoveZero(self.task, self.lLimb, self.rLimb, self.pause_event)
            self.rawCommand = ""
            self.update_command_box("Moving to zero")


        self.lastAliveStatus = self.task.is_alive()
        self.lastAliveName = self.task.name
        #at the end
        self.newCommand = ""
        self.lastCommand = self.command
        time.sleep(.01)

        #set env variables


   # def set_env_variables(self):


    def run(self):
        print "running program"
        self.setup_baxter()
        self.setup_speech_recognition()
        self.get_command()

    #initialize the app
    def __init__(self, master=None):
            Frame.__init__(self, master)
            self.pack()
            self.createWidgets()
            self.run()

root = Tk()
app = Application(master=root)
root.title("Baxter Kitchen Helper")
root.minsize(width=600, height=800)
root.maxsize(width=600, height=800)
app.mainloop()
#end while loop

