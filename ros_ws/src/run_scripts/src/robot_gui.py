#!/usr/bin/env python

############# Baxter Speech Arm Positional Control ################

## Written by the Robotic Assistance research team
## Tufts University

## 01/2018

###################################################################
# 
# Uses a combination of user provided voice commands and pre-programmed movements
# to control the Baxter Robot from Rethink Robotics
# The movements are found in the package "taskFunctions.py"
# This program utilizes a number of packages including threading, speechRecognition, 
# and the baxter_interface package from Rethink. 

#Suggestion after timed cook!!
# booleans are wrong for microwave being on and off
# make boolean for where the food is

from GUIhelper import *
from Tkinter import *



#interface widgets#
#Dialog title
class robot_gui:

	def __init__(self):

		self.root = Tk()
		self.root['bg'] = "#ccefff"
		self.dialog_label = Label(self.root, text="Dialog", fg="black", bg="light blue", 
		                     width=60, height=2, font=("Ariel", 20))
		self.dialog_label.pack()

		#Dialog box for displaying received voice commands
		self.dialog_box = Label(self.root, bg="white", padx=10, pady=10, 
		                   borderwidth=1, relief=SOLID, anchor="nw",
		                   width=100, height=4, font=("Ariel", 15),
		                   wraplength=500, justify=LEFT)
		self.dialog_content = StringVar()
		self.dialog_content.set("Listening...")
		self.dialog_box["textvariable"] = self.dialog_content
		self.dialog_box.pack(fill = X)


		#command label for displaying interpreted commands
		self.command_box = Label(self.root, fg="black", bg="lightgrey",
		                    bd=1, relief=SOLID, width=60, height=2, 
		                    font=("Courier New", 19), wraplength=500)
		self.curr_command = StringVar()
		self.curr_command.set("Waiting...")
		self.command_box["textvariable"] = self.curr_command
		self.command_box.pack()
		#blank label separator

		#box for displaying suggestions
		self.prompt = Frame(self.root, width=100, height=200, bg="#ccefff")
		self.prompt.pack()
		self.sug_header = Label(self.prompt, text="Suggested Commands", font=("Ariel",18), bg="#ccefff",
		                    fg="#000033")
		self.sug_header.pack(pady=10)
		self.first_suggestion = StringVar()
		self.first = Label(self.prompt, textvariable=self.first_suggestion, font = ("Ariel", 15), bg="#ccefff",
		              fg="#181818")
		self.first.pack(pady=5)
		self.second_suggestion = StringVar()
		self.second = Label(self.prompt, textvariable=self.second_suggestion, font = ("Ariel", 15), bg="#ccefff",
		                fg="#181818")
		self.second.pack(pady=10)
		# self.env_frame = Frame(self.root, width=200, height=100, bg="#ccefff")
		# self.env_frame.pack(pady=20)
		# self.microwave_value = StringVar()
		# self.microwave_value.set("Microwave: Off, Closed")
		# self.microwave = Label(self.env_frame, textvariable=self.microwave_value, bg="light grey", bd=1, 
		#                   relief=SOLID, width=100, height=100, font=("Ariel", 15))
		# self.microwave.pack(padx= 10, side=LEFT, pady=5)

		#Displays whether or not robot is localized
		self.mode_value = StringVar()
		self.mode_value.set("Robot not localized")
		self.robot_mode = Label(self.root, textvariable=self.mode_value, fg="white", bg="red", 
		width=60, height=2, font=("Ariel", 20), bd=1, relief=SOLID)
		self.robot_mode.pack()

		#environment title. Aesthetics for the label "Environment"
		self.env_label = Label(self.root, text="Environment", fg="white", bg="#000d33",
		                  width=60, height=2, font=("Ariel", 20), bd=1, relief=SOLID)
		self.env_label.pack()

		self.env_frame = Frame(self.root, width=200, height=100, bg="#ccefff")
		self.env_frame.pack(pady=20)

		
		#Fridge box - Indicates the state of the fridge
		
		self.fridge_value = StringVar()
		self.fridge_value.set("Fridge: Closed")
		self.fridge = Label(self.env_frame, textvariable=self.fridge_value, bg="light grey", bd=1, relief=SOLID, 
		                width=20, height=6, font=("Ariel", 15))
		self.fridge.pack(padx = 10, side=LEFT, pady=5)

		
		#Microwave box - Indicates the state of the microwave
		self.microwave_value = StringVar()
		self.microwave_value.set("Microwave: Off, Closed")
		self.microwave = Label(self.env_frame, textvariable=self.microwave_value, bg="light grey", bd=1, 
		                  relief=SOLID, width=20, height=6, font=("Ariel", 15))
		self.microwave.pack(padx= 10, side=LEFT, pady=5)
		
		#Aesthetics
		self.env_frame2 = Frame(self.root, width=200, height=100, bg="#ccefff")
		self.env_frame2.pack(pady=20)

		
		#Bottle box - Indicates the state of the bottle
		self.bottle_value = StringVar()
		self.bottle_value.set("Bottle: In fridge")
		self.bottle = Label(self.env_frame2, textvariable=self.bottle_value, width=20, height=6, bd=1, relief=SOLID,
		                font=("Ariel", 15), bg="light grey")
		self.bottle.pack(padx= 10, side=LEFT, pady=5) 
		
		
		# #Food box - Indicates the state of the food
		self.food_value = StringVar()
		self.food_value.set("food: In fridge")
		self.food = Label(self.env_frame2, textvariable=self.food_value, width=20, height=6, bd=1, relief=SOLID,
		                font=("Ariel", 15), bg="light grey")
		self.food.pack(padx= 10, side=LEFT, pady=5) 
		
		# #environment tokens#
		self.root.title("Baxter Kitchen Helper")
		self.root.minsize(width=800, height=1000)
		self.root.maxsize(width=800, height=1000)
		
		self.label_dict={
		'dialog':'Listening...',
		'command':'Waiting...'
		}


	def update_root(self):
		self.root.update()

	def main_loop(self):
		self.root.mainloop()

	def update_command_display(self):
		self.dialog_content.set(self.label_dict['dialog'])
		
		self.curr_command.set(self.label_dict['command'])
		


	def update_labels(self, env):

		if env['robotlocalized']:
			self.mode_value.set("Robot Localized")
			self.robot_mode['bg'] = 'green'

		if env['foodInFridge']:
			self.food_value.set("Food: in Fridge")
		elif env['foodInMicrowave']:
			self.food_value.set("Food: in Microwave")
			self.food['bg'] = 'light green'
		elif env['foodOnTable']:
			self.food_value.set("Food: On Table")


		if (env['hasBottle']):
			self.bottle_value.set("Bottle: In hand")
			self.bottle['bg'] = 'light green'
		if (env['bottleOnTable']):
			self.bottle_value.set("Bottle: On table")
			self.bottle['bg'] = 'light grey'


		if(env['fridgeOpen']):
			self.fridge_value.set("Fridge: Open")
			self.fridge['bg'] = "light green"
		if(not env['fridgeOpen']):
			self.fridge_value.set("Fridge: Closed")
			self.fridge['bg'] = "light grey"


		if(env['microwaveOn'] and env['microwaveOpen']):
			self.microwave_value.set("Microwave: On, Open")
			self.microwave['bg'] = "light green"
		elif (env['microwaveOpen'] and not env['microwaveOn']):
			self.microwave_value.set("Microwave: Off, Open")
			self.microwave['bg'] = "light green"
		elif (not env['microwaveOpen'] and env['microwaveOn']):
			self.microwave_value.set("Microwave: On, Closed")
			self.microwave['bg'] = "light green"
		elif (not env['microwaveOpen'] and not env['microwaveOn']):
			self.microwave_value.set("Microwave: Off, Closed")
			self.microwave['bg'] = "light grey"

		 

#g = robot_gui()
#g.main_loop() 
# while True:
# 	g.update_root()