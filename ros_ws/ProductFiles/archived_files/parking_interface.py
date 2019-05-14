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

from Tkinter import *
from math import *



#interface widgets#
#Dialog title
class robot_gui:

	def __init__(self):
		self.standard_x_dist = 1.0
		self.standard_y_dist = 0.8
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
		# self.prompt = Frame(self.root, width=100, height=200, bg="#ccefff")
		# self.prompt.pack()
		# self.sug_header = Label(self.prompt, text="Suggested Commands", font=("Ariel",18), bg="#ccefff",
		#                     fg="#000033")
		# self.sug_header.pack(pady=10)
		# self.first_suggestion = StringVar()
		# self.first = Label(self.prompt, textvariable=self.first_suggestion, font = ("Ariel", 15), bg="#ccefff",
		#               fg="#181818")
		# self.first.pack(pady=5)
		# self.second_suggestion = StringVar()
		# self.second = Label(self.prompt, textvariable=self.second_suggestion, font = ("Ariel", 15), bg="#ccefff",
		#                 fg="#181818")
		# self.second.pack(pady=10)
		# self.env_frame = Frame(self.root, width=100, height=100, bg="#ccefff")
		# self.env_frame.pack(pady=20)
		# self.microwave_value = StringVar()
		# self.microwave_value.set("Microwave: Off, Closed")
		# self.microwave = Label(self.env_frame, textvariable=self.microwave_value, bg="light grey", bd=1, 
		#                   relief=SOLID, width=20, height=10, font=("Ariel", 15))
		# self.microwave.pack(padx= 10, side=LEFT, pady=5)

	#Displays whether or not robot is localized
		self.mode_value = StringVar()
		self.mode_value.set("Robot In Position")
		self.robot_mode = Label(self.root, textvariable=self.mode_value, fg="white", bg="blue", 
		width=60, height=1, font=("Ariel", 15), bd=1, relief=SOLID)
		self.robot_mode.pack()


		self.canvas_width = 120
		self.canvas_height = 120
		robot_dimen = 100
		self.canvas = Canvas(self.root, width=self.canvas_width, height=self.canvas_height, bg='white')
		self.robot = self.canvas.create_rectangle(10,10,110,110,fill='black')
		self.canvas.pack(pady=15)
		pos = self.canvas.coords(self.robot)
		self.origin={'x':pos[2],'y':pos[3]}
		#Displays whether or not robot is localized
		# self.mode_value = StringVar()
		# self.mode_value.set("Robot not localized")
		# self.robot_mode = Label(self.root, textvariable=self.mode_value, fg="white", bg="red", 
		# width=60, height=2, font=("Ariel", 20), bd=1, relief=SOLID)
		# self.robot_mode.pack()

		# #environment title. Aesthetics for the label "Environment"
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

	#10cm = 1 pixel
	def dist_to_pix(self,delta):
		return delta*10

	def update_command_display(self):
		self.dialog_content.set(self.label_dict['dialog'])
		
		self.curr_command.set(self.label_dict['command'])
		
	def adjust_robot(self, env):
		#print('about to adjust')
		x_dist = env['distanceToKitchen']
		y_dist = env['distanceToTable']

		if (-0.03 < x_dist - self.standard_x_dist < 0.03) and (-0.03 < y_dist - self.standard_y_dist < 0.03):
			pos = self.canvas.coords(self.robot)
			d_x = self.origin['x'] - pos[2]
			d_y = self.origin['y'] - pos[3] 
			self.canvas.move(self.robot, d_x, d_y)
			self.mode_value.set("Robot In Position")
			self.robot_mode['bg'] = 'blue'
			#print('nothing')
			return 


		if x_dist - self.standard_x_dist> 0.03:
			pos = self.canvas.coords(self.robot)
			delta_x = self.origin['x'] - pos[2] -5
			self.canvas.move(self.robot, delta_x,0)
			self.mode_value.set("Robot Not In Position")
			self.robot_mode['bg'] = 'red'
			#print('move left')

		elif x_dist - self.standard_x_dist < -0.03:
			pos = self.canvas.coords(self.robot)
			delta_x = self.origin['x'] - pos[2] + 5
			self.canvas.move(self.robot, delta_x, 0)
			self.mode_value.set("Robot Not In Position")
			self.robot_mode['bg'] = 'red'
			#print('move right')

		if y_dist - self.standard_y_dist> 0.03:
			pos = self.canvas.coords(self.robot)
			delta_y = self.origin['y'] - pos[3] - 5
			self.canvas.move(self.robot, 0, delta_y)
			#print ('move forward')
			self.mode_value.set("Robot Not In Position")
			self.robot_mode['bg'] = 'red'

		elif y_dist - self.standard_y_dist < -0.03:
			pos = self.canvas.coords(self.robot)
			delta_y = self.origin['y'] - pos[3] + 5
			self.canvas.move(self.robot, 0, delta_y)
			#print('move backward')
			self.mode_value.set("Robot Not In Position")
			self.robot_mode['bg'] = 'red'



	def update_labels(self, env):
		self.adjust_robot(env)
		#if env['robotlocalized']:
			#self.mode_value.set("Robot Localized")
			#self.robot_mode['bg'] = 'green'

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
