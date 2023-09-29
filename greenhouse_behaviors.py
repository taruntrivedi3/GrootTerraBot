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
        self.moisture_opt = optimal["moisture"][0]

        # STUDENT CODE: Modify these lines to use your own initial state name
        #               and add all your FSM states
        self.initial = 'initial'
        self.states = [self.initial,'initialWaiting','waiting','done','watering','measuring']
        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition('enable',self.initial,'initialWaiting',after='setInitial')
        self.fsm.add_transition('doStep','initialWaiting','initialWaiting',conditions = ['nextDay'],after="resetTotalWater")
        self.fsm.add_transition('doStep','initialWaiting','waiting',conditions = ['waiting'])
        self.fsm.add_transition('doStep','waiting','done',conditions = ['enoughWater'],after="printWateredEnough")
        self.fsm.add_transition('doStep','waiting','waiting',conditions = ['waterLevelLow'],after="notEnoughWater")
        self.fsm.add_transition('doStep','waiting','done',conditions = ['moistEnough'],after="moistEnoughPrint")
        self.fsm.add_transition('doStep','waiting','watering',conditions = ['tooDry'],after="wateringAction")  
        self.fsm.add_transition('doStep','watering','measuring',conditions = ['waiting'],after="postWateringToMeasuring") 
        self.fsm.add_transition('doStep','measuring','waiting',conditions = ['waiting'],after="calcWaterAdded")  
        self.fsm.add_transition('disable','waiting',self.initial,after="setInitial")
        self.fsm.add_transition('disable','done',self.initial,after="setInitial")
        self.fsm.add_transition('disable','watering',self.initial,after="setInitial")
        self.fsm.add_transition('disable','measuring',self.initial,after="setInitial")
             
        # END STUDENT CODE

    def setInitial(self):
    	#originially pass
        self.perceive()
        self.setPump(False)
        self.setTimer(10)
        
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
    def nextDay(self):
    	return self.last_time > self.mtime
    	
    def waiting(self):
    	return self.time >= self.waittime
    	
    def enoughWater(self):
    	return self.total_water >= self.daily_limit
    	
    def waterLevelLow(self):
    	return self.water_level < 30
    
    def tooDry(self):  return self.smoist_est < self.moisture_opt
    
    def moistEnough(self): return self.smoist_est >= self.moisture_opt
    # END STUDENT CODE
        
    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def resetTotalWater(self): # Reset total water each day
        print("Resetting total water")
        self.total_water = 0
        self.setLastTime()

    def setLastTime(self): self.last_time = self.mtime
    
    def setTimer(self, wait):
        self.waittime = self.time + wait
    
    def notEnoughWater(self): print('NOT ENOUGH WATER IN RESERVOIR')
    
    def moistEnoughPrint(self): print("Soil is moist enough (%s)" %self.smoist_est)
    
    def calcWaterAdded(self):
        dwater = self.weight_est - self.start_weight # ml of water weighs a gram
        # Sometimes scales are off - cannot lose weight after watering
        dwater = max(0, dwater)

        self.total_water += dwater
        print("calcWaterAdded: %.1f (%.1f = %.1f - %.1f)"
              %(self.total_water, dwater, self.weight_est, self.start_weight))
    
    def postWateringToMeasuring(self):
    	self.setPump(False); self.setTimer(20)
	
    def wateringAction(self):
    	print("Soil too dry (%s) - need to water" %self.smoist_est)
    	self.start_weight = self.weight_est; self.setTimer(10); self.setPump(True)
    
    
    	
    
    def printWateredEnough(self):
        print("Watered Enough: %.1f" %self.total_water)
        
  

	
	
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

