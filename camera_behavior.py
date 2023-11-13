from behavior import *
from transitions import Machine
import sys, os.path as op
import os
from terrabot_utils import clock_time
import time

'''
The behavior should adjust the lights to a reasonable level (say 400-600),
wait a bit for the light to stabilize, and then request an image.
It should check to be sure the image has been recorded and, if so, process
the image; if not, try again for up to 3 times before giving up
'''
class TakeImage(Behavior):
    def __init__(self):
        super(TakeImage, self).__init__("TakeImageBehavior")
        # Your code here
	# Initialize the FSM and add transitions
        # BEGIN STUDENT CODE
        self.retry_cnt = 0
        self.img_cnt = 0
        self.last_time = 24*60*60
        self.pathname = ""
        self.initial = 'halt'
        self.directory = '/home/robotanist/Desktop/TerraBot/pictures'
        self.modified_directory = '/home/robotanist/Desktop/TerraBot/modified_pictures'
        self.states = [self.initial, 'init', 'light', 'check', 'recheck', 'done']
        
        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)
                           
        self.fsm.add_transition('enable', self.initial, 'init', after=['setInitial'])
        
        self.fsm.add_transition('disable', 'init', self.initial, after=['setEnd'])
        self.fsm.add_transition('disable', 'light', self.initial, after=['setEnd'])
        self.fsm.add_transition('disable', 'check', self.initial, after=['setEnd'])
        self.fsm.add_transition('disable', 'recheck', self.initial, after=['setEnd'])
        self.fsm.add_transition('disable', 'done', self.initial, after=['setEnd'])
        
        #self.fsm.add_transition('doStep', 'init', 'done', conditions=['moreThan3Pics'], after=[])
        
        self.fsm.add_transition('doStep', 'init', 'init', conditions=['isNextDay'], after=['nextDay'])
        self.fsm.add_transition('doStep', 'init', 'init', conditions=['noDirExist'], after=['createDir'])
        self.fsm.add_transition('doStep', 'init', 'light', conditions=['isToday'], after=['setTimer5'])
        
        self.fsm.add_transition('doStep', 'light', 'light', conditions=['isLight2low', 'isTimeUp'], after=['raiseLight', 'setTimer5'])
        self.fsm.add_transition('doStep', 'light', 'light', conditions=['isLight2high', 'isTimeUp'], after=['lowerLight', 'setTimer5'])
        self.fsm.add_transition('doStep', 'light', 'check', conditions=['isLightGood', 'isTimeUp', 'noMoreThan3Pics', 'dirExist'], after=['acquireImg', 'setTimer10'])
        self.fsm.add_transition('doStep', 'light', 'done', conditions=['isLightGood', 'isTimeUp', 'noMoreThan3Pics', 'noDirExist'], after=['raiseError'])
        self.fsm.add_transition('doStep', 'light', 'done', conditions=['moreThan3Pics'])
        
        self.fsm.add_transition('doStep', 'check', self.initial, conditions=['fileExist', 'isTimeUp'], after=['setInitial', 'addCnt'])
        self.fsm.add_transition('doStep', 'check', 'recheck', conditions=['noFileExist', 'isTimeUp', 'noMoreThan3Pics', 'dirExist'], after=['reacquireImg', 'setTimer20'])
        self.fsm.add_transition('doStep', 'check', 'done', conditions=['noFileExist', 'isTimeUp', 'noMoreThan3Pics', 'noDirExist'], after=['raiseError'])
        self.fsm.add_transition('doStep', 'check', 'done', conditions=['moreThan3Pics'])
        
        self.fsm.add_transition('doStep', 'recheck', self.initial, conditions=['fileExist', 'isTimeUp'], after=['setInitial', 'addCnt'])
        self.fsm.add_transition('doStep', 'recheck', 'recheck', conditions=['noFileExist', 'isTimeUp', 'noMoreThan3Trials', 'noMoreThan3Pics', 'dirExist'], after=['reacquireImg', 'setTimer20'])
        self.fsm.add_transition('doStep', 'recheck', 'done', conditions=['noFileExist', 'isTimeUp', 'noMoreThan3Trials', 'noMoreThan3Pics', 'noDirExist'], after=['raiseError'])       
        self.fsm.add_transition('doStep', 'recheck', 'done', conditions=['moreThan3Pics'])
        self.fsm.add_transition('doStep', 'recheck', self.initial, conditions=['noFileExist', 'isTimeUp', 'moreThan3Trials'], after=['printWarning', 'setInitial'])
        # END STUDENT CODE

    # Add the condition and action functions
    #  Remember: if statements only in the condition functions;
    #            modify state information only in the action functions
    # BEGIN STUDENT CODE
    # Condition Functions
    def isNextDay(self):
        return self.last_time > self.mtime
    def isToday(self):
        return self.last_time <= self.mtime
    def isTimeUp(self):
        return self.time >= self.waittime
    
    def isLight2low(self):
        return self.light < 400        
    def isLight2high(self):
        return self.light >= 600
    def isLightGood(self):
        return (self.light >= 400) and (self.light < 600)
    
    def noMoreThan3Pics(self):
        return self.img_cnt < 3
    def moreThan3Pics(self):
        return self.img_cnt >= 3
    
    def fileExist(self):
        return op.exists(self.pathname)
    def noFileExist(self):
        return not(op.exists(self.pathname))
    
    def dirExist(self):
        return op.exists(self.directory)
    def noDirExist(self):
        return not(op.exists(self.directory))
    
    def noMoreThan3Trials(self):
        return self.retry_cnt <= 2
    def moreThan3Trials(self):
        return self.retry_cnt > 2
    
    # Action Functions
    def setInitial(self):
        self.retry_cnt = 0
        self.led = 0
        self.setLED(self.led) 
    def setEnd(self):
        self.retry_cnt = 0
        self.led = 0
        self.setLED(self.led) 
        self.setLastTime()
    
    def raiseLight(self):
        self.setLED(self.led+20)   
    def lowerLight(self):
        self.setLED(self.led-20)
    def setLED(self, level):
        self.led = max(0, min(255, level))
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"led": self.led}))  
    
    def setTimer(self, wait):
        self.waittime = self.time + wait
    def setTimer5(self): 
        self.setTimer(5)
    def setTimer10(self): 
        self.setTimer(10)
    def setTimer20(self): 
        self.setTimer(20)
    def setLastTime(self): 
        self.last_time = self.mtime
    
    def nextDay(self):
        print("Next day, reset image counter to 0!")
        self.img_cnt = 0
        self.setLastTime() 
    
    def createDir(self):
        os.mkdir(self.directory)
        os.mkdir(self.modified_directory)
    def acquireImg(self):
        name = self.directory + '/pic' + str(int(self.time)) + '.jpg'
        self.pathname = name
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"camera": name}))  
        print("Taking Image, and Light Level is (%s)" %self.light)                        
    def reacquireImg(self):
        name = self.directory + '/pic' + str(int(self.time)) + '.jpg'
        self.pathname = name
        self.retry_cnt += 1
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"camera": name}))  
        print("Retry taking Image, and Light Level is (%s)" %self.light)                                                        
    def addCnt(self):
        self.img_cnt += 1
        print("Have taken (%s) images" %self.img_cnt)
                              
    def printWarning(self):
        print("Warning: Tried to acquire the image unsuccessfully three times in a row!") 
    def raiseError(self):
        print("Error: Writing to a non-existent directory!")  
    # END STUDENT CODE

    def perceive(self):
        self.time = self.sensordata['unix_time']
        # Add any sensor data variables you need for the behavior
        # BEGIN STUDENT CODE
        self.light = self.sensordata['light']
        self.mtime = self.sensordata['midnight_time']
        # END STUDENT CODE

    def act(self):
        self.trigger("doStep")
