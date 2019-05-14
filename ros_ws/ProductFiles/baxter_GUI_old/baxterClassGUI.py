from Tkinter import *

root = Tk()
root.title("Baxter Kitchen Helper")
root.minsize(width=600, height=800)
root.maxsize(width=600, height=800)

#Dialog title
dialog_label = Label(root, text="Dialog", fg="black", bg="light blue", 
        		     width=60, height=2, font=("Ariel", 20))
dialog_label.pack()

#Dialog box
dialog_box = Label(root, bg="white", padx=10, pady=10, 
        		   borderwidth=1, relief=SOLID, anchor="nw",
        		   width=100, height=4, wraplength=300,
        		   font=("Ariel", 15))
dialog_content = StringVar()
dialog_content.set("Listening...")
dialog_box["textvariable"] = dialog_content
dialog_box.pack(fill = X)

#command label
command_label = Label(root, text="Current Task", fg="black", bg="light blue", 
        							width=60, height=2, font=("Ariel", 20))
command_label.pack()   
command_box = Label(root, fg="black", bg="lightgrey",
       				bd=1, relief=SOLID, width=60, height=2, 
       				font=("Ariel", 20))
command = StringVar()
command.set("Waiting...")
command_box["textvariable"] = command
command_box.pack()

#blank label separator
#blank = Label(root, width=60, height=2, bg = "light blue")
#blank.pack(fill = X)

#environment title
env_label = Label(root, text="Environment", fg="black", bg="light blue", 
        							width=60, height=2, font=("Ariel", 20))
env_label.pack()        

#environment tokens
env1 = Label(root, text="ENV-1", bg="dark grey",
        	bd=1, relief=SOLID, width=60, height=3,
        	font=("Ariel", 18))
env1.pack()
env2 = Label(root, text="ENV-2", bg="dark grey",
        	bd=1, relief=SOLID, width=60, height=3,
        	font=("Ariel", 18))
env2.pack()

env3 = Label(root, text="ENV-3", bg="dark grey",
        	bd=1, relief=SOLID, width=60, height=3,
        	font=("Ariel", 18))
env3.pack()

env4 = Label(root, text="ENV-4", bg="dark grey",
        	bd=1, relief=SOLID, width=60, height=3,
        	font=("Ariel", 18))
env4.pack()

#env5 = Label(root, text="ENV-4", bg="dark grey",
 #       	bd=2, relief=SOLID, width=60, height=3,
  #      	font=("Ariel", 18))
#env5.pack()

root.mainloop()
#end while loop