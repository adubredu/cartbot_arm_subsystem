from multiprocessing import Process
from threading import (
    Thread,
    activeCount,
    Event as TEvent,
    enumerate,
)
from positionControl import *
from taskFunctions import *


def runMoveZero(task, lLimb, rLimb, pause_event):
	terminate_thread(task)
	task = Thread(target=moveToDownward, args=(lLimb, rLimb, pause_event),
				  name="movingToZero")
	print(task.name)
	pause_event.clear()
	task.daemon = True
	task.start()

def terminate_thread(thread):
	if not thread.isAlive():
		return
	exc = ctypes.py_object(KeyboardInterrupt)
	res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread.ident), exc)
	if res == 0:
		raise ValueError("nonexistent thread id")
	elif res > 1:
		ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, None)
		raise SystemError("PyThreadState_SetAsyncExc failed")
