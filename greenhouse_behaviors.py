from behavior import *
from limits import *
from transitions import Machine
import logging


#sensor data passed into greenhouse behaviors:
#  [time, lightlevel, temperature, humidity, soilmoisture, waterlevel]
#actuators are looking for a dictionary with any/all of these keywords:
#  {"led":val, "fan":True/False, "pump": True/False}


'''
The combined ambient and LED light level between 8am and 10pm should be 
in the optimal['light_level'] range;
Between 10pm and 8am, the LEDs should be off (set to 0).
'''
class Light(Behavior):

    def __init__(self):
        super(Light, self).__init__("LightBehavior")
        self.optimal_level = optimal['light_level']
        

        # STUDENT CODE: Modify these lines to use your own initial state name
        #               and add all your FSM states
        self.initial = 'initial'
        self.states = [self.initial,'light','dark','initialWaiting']
        
        

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        
        self.fsm.add_transition('enable',self.initial,'initialWaiting',after="setInitial")
        self.fsm.add_transition('enable', self.initial,'initialWaiting',after="setInitial")
        self.fsm.add_transition('doStep','initialWaiting', 'light', conditions=['goodHours'],after = "setLEDGood")
        self.fsm.add_transition('doStep','initialWaiting','dark',conditions=['badHours'],after='setLEDBad')
        self.fsm.add_transition('doStep', 'light','light',conditions=['goodHours'],after = "setLEDGood")
        self.fsm.add_transition('doStep', 'light','dark',conditions=['badHours'],after = "setLEDBad")
        self.fsm.add_transition('doStep','dark','light',conditions=['goodHours'],after = "setLEDGood")
        self.fsm.add_transition('doStep', 'dark','dark',conditions=['badHours'],after = "setLEDBad")
        self.fsm.add_transition('disable','dark',self.initial,after="setInitial")
        self.fsm.add_transition('disable','light',self.initial,after="setInitial")
        # END STUDENT CODE
        
    def setInitial(self):
        self.led = 0
        self.setLED(self.led)
    def setOptimal(self, new_optimal):
    	self.optimal_level[0] = new_optimal - 40
    	self.optimal_level[1] = new_optimal + 40 
    	
    def perceive(self):
        self.mtime = self.sensordata["midnight_time"]
        self.time = self.sensordata["unix_time"]
        self.light = self.sensordata["light"]
    
    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")
        
    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def goodHours(self):
    	#print('hello')
    	self.perceive()
    	hour = (self.mtime//3600)%24
    	print(hour)
    	return hour >= 8 and hour < 22
    	
        
    	
            
    def badHours(self):
    	self.perceive()
    	hour = (self.mtime//3600)%24
    	return not (hour >= 8 and hour < 22)
    	
      
    # END STUDENT CODE
        
    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def setLEDGood(self):
    	if self.light < self.optimal_level[0]:
                self.setLED(self.led+20); print('setting LED Good')
    	elif self.light >= self.optimal_level[1]:
                self.setLED(self.led-20)
    	
        
    def setLEDBad(self):
    	self.setLED(0)
    # END STUDENT CODE
    


    def setLED(self, level):
        self.led = max(0, min(255, level))

        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"led": self.led}))
                                  
                                  

"""
The temperature should be greater than the lower limit
"""
class RaiseTemp(Behavior):

    def __init__(self):
        super(RaiseTemp, self).__init__("RaiseTempBehavior")

        # STUDENT CODE: Modify these lines to use your own initial state name
        #               and add all your FSM states
        self.initial = 'initial'
        self.states = [self.initial,'tooLow','perfect']

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition('enable',self.initial,'tooLow',conditions = ['lowTemp'],after="setInitial")
        self.fsm.add_transition('enable', self.initial,'perfect', conditions = ['goodTemp'],after="setInitial")
        self.fsm.add_transition('doStep', 'tooLow','tooLow',conditions=['lowTemp'],after = "setLEDlowTemp")
        self.fsm.add_transition('doStep', 'tooLow','perfect',conditions=['goodTemp'],after = "setLEDgoodTemp")
        self.fsm.add_transition('doStep','perfect','tooLow',conditions = ['lowTemp'],after="setLEDlowTemp")
        self.fsm.add_transition('doStep', 'perfect','perfect',conditions=['goodTemp'],after = "setLEDgoodTemp")
        self.fsm.add_transition('disable','tooLow',self.initial,after="setInitial")
        self.fsm.add_transition('disable','perfect',self.initial,after="setInitial")
        
        # END STUDENT CODE

    def setInitial(self):
        self.setLED(0)
        
    def perceive(self):
        self.temp = self.sensordata["temp"]
        print(self.sensordata.keys())
        #self.light = self.sensordata["light"]
        

    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")

    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def lowTemp(self):
    	self.perceive()
    	return self.temp < limits['temperature'][0]
    
    def goodTemp(self):
    	self.perceive()
    	return self.temp >= optimal['temperature'][0]
    	
    	
    # END STUDENT CODE

    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    
    def setLEDgoodTemp(self):
    	self.setLED(0)
    	print("Temperature is now perfect!")
    def setLEDlowTemp(self): self.setLED(200); print("Turning up the lights to raise the temperature")
    	
                
                
                
    
    # END STUDENT CODE
            
    def setLED(self, level):
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"led": level}))
        
"""
The temperature should be less than the upper limit
"""
class LowerTemp(Behavior):

    def __init__(self):
        super(LowerTemp, self).__init__("LowerTempBehavior")

        # STUDENT CODE: Modify these lines to use your own initial state name
        #               and add all your FSM states
        self.initial = 'initial'
        self.states = [self.initial,'tooHigh','perfect','initialWaiting']

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition('enable',self.initial,'initialWaiting',after="setInitial")
        self.fsm.add_transition('enable', self.initial,'initialWaiting',after="setInitial")
        self.fsm.add_transition('doStep','initialWaiting','tooHigh',conditions = ['intoTooHigh'],after="tooHighAction")
        self.fsm.add_transition('doStep', 'initialWaiting','perfect', conditions = ['intoPerfect'],after="perfectAction")
        self.fsm.add_transition('doStep', 'perfect','tooHigh',conditions=['intoTooHigh'],after = "tooHighAction")
        self.fsm.add_transition('doStep', 'tooHigh','perfect',conditions=['intoPerfect'],after = "perfectAction")
        #self.fsm.add_transition('doStep','perfect','tooLow',conditions = ['lowTemp'],after="setLEDlowTemp")
        #self.fsm.add_transition('doStep', 'perfect','perfect',conditions=['goodTemp'],after = "setLEDgoodTemp")
        self.fsm.add_transition('disable','tooHigh',self.initial,after="setInitial")
        self.fsm.add_transition('disable','perfect',self.initial,after="setInitial")
        # END STUDENT CODE
        

    def setInitial(self):
        self.setFan(False)
        
    def perceive(self):
        self.temp = self.sensordata["temp"]

    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")

    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def intoTooHigh(self):
    	return self.temp >= limits['temperature'][1]
    
    def intoPerfect(self):
    	return self.temp <= optimal['temperature'][1]
    
    # END STUDENT CODE
        
    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def tooHighAction(self):
    	self.setFan(True)
    	print("Turning on the fan to lower temperature")
    
    def perfectAction(self):
    	self.setFan(False)
    	print("Temperature is now perfect!")
    # END STUDENT CODE
            
    def setFan(self, act_state):
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"fan": act_state}))
    
"""
Humidity should be less than the limit
"""
class LowerHumid(Behavior):

    def __init__(self):
        super(LowerHumid, self).__init__("LowerHumidBehavior")

        # STUDENT CODE: Modify these lines to use your own initial state name
        #               and add all your FSM states
        self.initial = 'initial'
        self.states = [self.initial,'tooHigh','perfect','initialWaiting']

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition('enable',self.initial,'initialWaiting',after="setInitial")
        self.fsm.add_transition('enable', self.initial,'initialWaiting',after="setInitial")
        self.fsm.add_transition('doStep','initialWaiting','tooHigh',conditions = ['intoTooHigh'],after="tooHighAction")
        self.fsm.add_transition('doStep', 'initialWaiting','perfect', conditions = ['intoPerfect'],after="perfectAction")
        self.fsm.add_transition('doStep', 'perfect','tooHigh',conditions=['intoTooHigh'],after = "tooHighAction")
        self.fsm.add_transition('doStep', 'tooHigh','perfect',conditions=['intoPerfect'],after = "perfectAction")
        #self.fsm.add_transition('doStep','perfect','tooLow',conditions = ['lowTemp'],after="setLEDlowTemp")
        #self.fsm.add_transition('doStep', 'perfect','perfect',conditions=['goodTemp'],after = "setLEDgoodTemp")
        self.fsm.add_transition('disable','tooHigh',self.initial,after="setInitial")
        self.fsm.add_transition('disable','perfect',self.initial,after="setInitial")
        # END STUDENT CODE
        
    def setInitial(self):
        self.setFan(False)
        
    def perceive(self):
        self.humid = self.sensordata["humid"]

    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")

    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def intoTooHigh(self):
    	return self.humid >= limits['humidity'][1]
    
    def intoPerfect(self):
    	return self.humid <= optimal['humidity'][1]
    	
    # END STUDENT CODE
        
    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def tooHighAction(self):
    	self.setFan(True)
    	print("Turning on the fan to lower humidity")
    
    def perfectAction(self):
    	self.setFan(False)
    	print("Humidity is now perfect!")
    # END STUDENT CODE

    def setFan(self, act_state):
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"fan": act_state}))
            
"""
Soil moisture should be greater than the lower limit
"""
class RaiseSMoist(Behavior):

    def __init__(self):
        super(RaiseSMoist, self).__init__("RaiseMoistBehavior")
        self.weight = 0
        self.weight_window = []
        self.smoist_window = []
        self.total_water = 0
        self.water_level = 0
        self.start_weight = 0
        self.last_time = 24*60*60 # Start with the prior day
        self.daily_limit = 50 #100
        self.watered = False
        self.addWater = False
        self.dayNum = 1
        self.moisture_opt = optimal["moisture"][0]

        # STUDENT CODE: Modify these lines to use your own initial state name
        #               and add all your FSM states

        self.initial = 'initial'
        self.states = [self.initial, 'init', 'waiting', 'watering', 'measuring', 'done']
        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)
        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition('enable', self.initial, 'init', after='setInit')
        
        self.fsm.add_transition('disable', 'init', self.initial, after='setEnd')
        self.fsm.add_transition('disable', 'waiting', self.initial, after='setEnd')
        self.fsm.add_transition('disable', 'watering', self.initial, after='setEnd')
        self.fsm.add_transition('disable', 'measuring', self.initial, after='setEnd')
        self.fsm.add_transition('disable', 'done', self.initial, after='setEnd')
        
        self.fsm.add_transition('doStep', 'init', 'init', conditions=['isNextDay', 'watered'], after='nextDay')
        self.fsm.add_transition('doStep', 'init', 'watering', conditions=['isNextDay', 'notWateredEarly'], after=['nextDayWaterEarly', 'setAddWater'])
        self.fsm.add_transition('doStep', 'init', 'watering', conditions=['isNextDay', 'notWateredMiddle'], after=['nextDayWaterMiddle', 'setAddWater'])
        self.fsm.add_transition('doStep', 'init', 'watering', conditions=['isNextDay', 'notWateredLAte'], after=['nextDayWaterLate', 'setAddWater'])
        #self.fsm.add_transition('doStep', 'init', 'watering', conditions=['isNextDay', 'notWatered'], after=['nextDayWater', 'setAddWater'])
        #self.fsm.add_transition('doStep', 'init', 'init', conditions=['isNextDay'], after='nextDay')
        self.fsm.add_transition('doStep', 'init', 'waiting', conditions=['isTimeUp'])
        self.fsm.add_transition('doStep', 'waiting', 'done', conditions=['isWateredEnough'], after='printWateredEnough')
        self.fsm.add_transition('doStep', 'waiting', 'done', conditions=['noEnoughWater'], after='emailNoWater') # qq
        self.fsm.add_transition('doStep', 'waiting', 'done', conditions=['smoistEnough'], after='printSmoistEnough')
        self.fsm.add_transition('doStep', 'waiting', 'watering', conditions=['s2Dry'], after=['startWatering', 'setWatered'])
        #self.fsm.add_transition('doStep', 'waiting', 'watering', conditions=['s2Dry'], after=['startWatering'])
        self.fsm.add_transition('doStep', 'watering', 'measuring', conditions=['isTimeUp'], after='startMeasuring')
        self.fsm.add_transition('doStep', 'measuring', 'waiting', conditions=['isTimeUp', 'notAddWater'], after='calcWaterAdded')
        self.fsm.add_transition('doStep', 'measuring', 'init', conditions=['isTimeUp', 'isAddWater'], after=['calcWaterAdded', 'setNoAddWater'])
    
             
        # END STUDENT CODE

    def setInitial(self):
    	pass
        
    def sliding_window(self, window, item, length=4):
        if (len(window) == length): window = window[1:]
        window.append(item)
        return window, sum(window)/float(len(window))
    
    def perceive(self):
        self.time = self.sensordata["unix_time"]
        self.mtime = self.sensordata["midnight_time"]
        self.water_level = self.sensordata["level"]
        self.weight = self.sensordata["weight"]
        self.weight_window, self.weight_est = self.sliding_window(self.weight_window, self.weight)
        self.smoist = self.sensordata["smoist"]
        self.smoist_window, self.smoist_est = self.sliding_window(self.smoist_window, self.smoist)

    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")

    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def watered(self):
        return self.watered
    
    def notWatered(self):
        return not(self.watered)
    
    def notWateredEarly(self):
        return not(self.watered) and self.dayNum<=3
    
    def notWateredMiddle(self):
        return not(self.watered) and 3<self.dayNum<=8
        
    def notWateredLate(self):   
        return not(self.watered) and self.dayNum>8
    
    def isAddWater(self):
        return self.addWater
    
    def notAddWater(self):
        return not(self.addWater)
        
    def isNextDay(self):
        return self.last_time > self.mtime
    
    def isTimeUp(self):
        return self.time >= self.waittime
    
    def isWateredEnough(self):
        return self.total_water >= self.daily_limit
    
    def noEnoughWater(self):
        return self.water_level < 30
    
    def smoistEnough(self):
        return self.smoist_est >= self.moisture_opt
    
    def s2Dry(self):
        return self.smoist_est < self.moisture_opt
    # END STUDENT CODE
        
    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def setInit(self):
        self.setPump(False)
        self.setTimer10()
    def setEnd(self):
        self.setPump(False)
        self.setLastTime()
    
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
        
    def resetTotalWater(self):
        print("Resetting total water")
        self.total_water = 0
        self.setLastTime()

    def calcWaterAdded(self):
        dwater = self.weight_est - self.start_weight
        dwater = max(0, dwater)

        self.total_water += dwater
        print("calcWaterAdded: %.1f (%.1f = %.1f - %.1f)"
              %(self.total_water, dwater, self.weight_est, self.start_weight))

	try:
		with open('water.txt', 'r') as file:
			daily_total = file.read()
		daily_total = daily_total.strip()
		daily_total = float(daily_total)
		daily_total += dwater
		with open('water.txt', 'w' as file:
			file.write(str(daily_total))
	except:
		pass
        
    def printWateredEnough(self):
        print("Watered Enough: %.1f" %self.total_water)

    def emailNoWater(self):
        print("NOT ENOUGH WATER IN RESERVOIR")
        
    def nextDay(self):
        print("Next day! Day: %s" %self.dayNum)
        print("Have watered yesterday!")
        self.watered = False
        self.addWater = False
        self.dayNum += 1
        self.resetTotalWater()
    
    def nextDayWater(self):
        print("Next day! Day: %s" %self.dayNum)
        self.watered = False
        self.startAddWatering()
        self.dayNum += 1 
        self.resetTotalWater()
   
    def nextDayWaterEarly(self):
        print("Next day! Day: %s" %self.dayNum)
        self.watered = False
        self.startAddWatering(6)
        self.dayNum += 1 
        self.resetTotalWater()  
    
    def nextDayWaterMiddle(self):
        print("Next day! Day: %s" %self.dayNum)
        self.watered = False
        self.startAddWatering(8)
        self.dayNum += 1 
        self.resetTotalWater()
    
    def nextDayWaterLate(self):
        print("Next day! Day: %s" %self.dayNum)
        self.watered = False
        self.startAddWatering(4)
        self.dayNum += 1 
        self.resetTotalWater()          
        
    def printSmoistEnough(self):
        print("Soil is moist enough (%s)" %self.smoist_est)
        
    def startWatering(self):
        print("Soil too dry (%s) - need to water" %self.smoist_est)
        self.start_weight = self.weight_est
        self.setTimer10()
        self.setPump(True)
        
    def startAddWatering(self, time):
        print("Haven't watered yet yesterday - need to water")
        self.start_weight = self.weight_est
        self.setTimer(time)
        self.setPump(True)
    
    def setWatered(self):
        self.watered = True
    def setAddWater(self):
        self.addWater = True
    def setNoAddWater(self):
        self.addWater = False
    
    def startMeasuring(self):
        self.setPump(False)
        self.setTimer20()
        
  
    # END STUDENT CODE
	
	
    def setPump(self,state):
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"wpump": state}))

"""
Soil moisture below the upper limit
"""
class LowerSMoist(Behavior):

    def __init__(self):
        super(LowerSMoist, self).__init__("LowerMoistBehavior")


        # STUDENT CODE: Modify these lines to use your own initial state name
        #               and add all your FSM states
        self.initial = 'initial'
        self.states = [self.initial,'tooHigh','perfect','initialWaiting']
       

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition('enable',self.initial,'initialWaiting',after="setInitial")
        self.fsm.add_transition('enable', self.initial,'initialWaiting',after="setInitial")
        self.fsm.add_transition('doStep','initialWaiting','tooHigh',conditions = ['intoTooHigh'],after="tooHighAction")
        self.fsm.add_transition('doStep', 'initialWaiting','perfect', conditions = ['intoPerfect'],after="perfectAction")
        self.fsm.add_transition('doStep', 'perfect','tooHigh',conditions=['intoTooHigh'],after = "tooHighAction")
        self.fsm.add_transition('doStep', 'tooHigh','perfect',conditions=['intoPerfect'],after = "perfectAction")
        #self.fsm.add_transition('doStep','perfect','tooLow',conditions = ['lowTemp'],after="setLEDlowTemp")
        #self.fsm.add_transition('doStep', 'perfect','perfect',conditions=['goodTemp'],after = "setLEDgoodTemp")
        self.fsm.add_transition('disable','tooHigh',self.initial,after="setInitial")
        self.fsm.add_transition('disable','perfect',self.initial,after="setInitial")
        # END STUDENT CODE
        
    def setInitial(self):
        self.setFan(False)
        
    def perceive(self):
        self.smoist = self.sensordata["smoist"]

    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")

    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def intoTooHigh(self): return self.smoist >= limits["moisture"][1]
    
    def intoPerfect(self): return self.smoist <= optimal['moisture'][1] 
    # END STUDENT CODE
        
    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def tooHighAction(self):self.setFan(True); print("Turning on the fan to lower soil moisture")
    
    def perfectAction(self): self.setFan(False); print("Soil moisture is now perfect!")
    
    
    # END STUDENT CODE
            
    def setFan(self, act_state):
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"fan": act_state}))

