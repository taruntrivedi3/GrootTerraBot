from behavior import *
from limits import *
from transitions import Machine

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
        self.states = [self.initial, 'night', 'day2low', 'day2high']

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        # self.actions = ['doStep', 'enable', 'disable']
        self.fsm.add_transition('enable', self.initial, 'night', after='setInitial')
        
        self.fsm.add_transition('disable', 'night', self.initial, after='setInitial')
        self.fsm.add_transition('disable', 'day2low', self.initial, after='setInitial')
        self.fsm.add_transition('disable', 'day2high', self.initial, after='setInitial')
        
        self.fsm.add_transition('doStep', 'night', 'day2low', conditions=['isDay2low'], after='raiseLight')
        self.fsm.add_transition('doStep', 'night', 'day2high', conditions=['isDay2high'], after='lowerLight')  
        self.fsm.add_transition('doStep', 'day2low', 'night', conditions = ['isNight'], after='setInitial')
        self.fsm.add_transition('doStep', 'day2high', 'night', conditions=['isNight'], after='setInitial')
        
        self.fsm.add_transition('doStep', 'day2low', 'day2high', conditions=['isDay2high'], after='lowerLight')
        self.fsm.add_transition('doStep', 'day2high', 'day2low', conditions=['isDay2low'], after='raiseLight')
        self.fsm.add_transition('doStep', 'day2high', 'day2high', conditions=['isDay2high'], after='lowerLight')
        self.fsm.add_transition('doStep', 'day2low', 'day2low', conditions=['isDay2low'], after='raiseLight')
        
        # END STUDENT CODE
        
    def setInitial(self):
        self.led = 0
        self.setLED(self.led)
        
    def perceive(self):
        self.mtime = self.sensordata["midnight_time"]
        self.time = self.sensordata["unix_time"]
        self.light = self.sensordata["light"]
    
    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")
        
    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def isNight(self):
        hour = (self.mtime//3600)%24
        return (hour < 8 or hour >= 22)
    
    def isDay2low(self):
        hour = (self.mtime//3600)%24
        return (hour >= 8 and hour < 22) and (self.light < self.optimal_level[0])
        
    def isDay2high(self):
        hour = (self.mtime//3600)%24
        return (hour >= 8 and hour < 22) and (self.light >= self.optimal_level[1])
    # END STUDENT CODE
        
    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def raiseLight(self):
    	self.setLED(self.led+20)
    
    def lowerLight(self):
    	self.setLED(self.led-20)
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
        self.states = [self.initial, 'temp2low', 'temp_perfect']

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition('enable', self.initial, 'temp_perfect', after='setInitial')
        
        self.fsm.add_transition('disable', 'temp2low', self.initial, after='setInitial')
        self.fsm.add_transition('disable', 'temp_perfect', self.initial, after='setInitial')
          
        self.fsm.add_transition('doStep', 'temp2low', 'temp_perfect', conditions = ['isTempPerfect'], after='doneLED')
        self.fsm.add_transition('doStep', 'temp_perfect', 'temp2low', conditions=['isTemp2low'], after='startLED')
        self.fsm.add_transition('doStep', 'temp_perfect', 'temp2low', conditions=['isTemp2low', 'LEDNotEnough'], after='startLED')
        self.fsm.add_transition('doStep', 'temp_perfect', 'temp2low', conditions=['isTemp2low', 'LEDEnough'])
        
        self.fsm.add_transition('doStep', 'temp_perfect', 'temp_perfect', conditions = ['isTempPerfect'], after='doneLED')
        self.fsm.add_transition('doStep', 'temp2low', 'temp2low', conditions=['isTemp2low'], after='startLED')
        self.fsm.add_transition('doStep', 'temp2low', 'temp2low', conditions=['isTemp2low', 'LEDNotEnough'], after='startLED')
        self.fsm.add_transition('doStep', 'temp2low', 'temp2low', conditions=['isTemp2low', 'LEDEnough'])
        # END STUDENT CODE

    def setInitial(self):
        self.setLED(0)
        
    def perceive(self):
        self.temp = self.sensordata["temp"]

    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")

    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def isTemp2low(self):
        return self.temp < limits['temperature'][0]
        
    def isTempPerfect(self):
        return self.temp >= optimal['temperature'][0]
        
    def LEDNotEnough(self):
        return self.sensordata['led'] < 200
    
    def LEDEnough(self):
        return self.sensordata['led'] >= 200
    # END STUDENT CODE

    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def startLED(self):
        self.setLED(200)
        print("Turning up the lights to raise the temperature")
        
    def doneLED(self):
        self.setLED(0)
        print("Temperature is now perfect!")
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
        self.states = [self.initial, 'temp2high', 'temp_perfect']

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition('enable', self.initial, 'temp_perfect', after='setInitial')
        
        self.fsm.add_transition('disable', 'temp2high', self.initial, after='setInitial')
        self.fsm.add_transition('disable', 'temp_perfect', self.initial, after='setInitial')
          
        self.fsm.add_transition('doStep', 'temp2high', 'temp_perfect', conditions = ['isTempPerfect'], after='doneFan')
        self.fsm.add_transition('doStep', 'temp_perfect', 'temp2high', conditions=['isTemp2high'], after='startFan')
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
    def isTemp2high(self):
        return self.temp >= limits['temperature'][1]
        
    def isTempPerfect(self):
        return self.temp <= optimal['temperature'][1]
    # END STUDENT CODE
        
    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def startFan(self):
        self.setFan(True)
        print("Turning on the fan to lower temperature")
        
    def doneFan(self):
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
        self.states = [self.initial, 'humid2high', 'humid_perfect']

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition('enable', self.initial, 'humid_perfect', after='setInitial')
        
        self.fsm.add_transition('disable', 'humid2high', self.initial, after='setInitial')
        self.fsm.add_transition('disable', 'humid_perfect', self.initial, after='setInitial')
          
        self.fsm.add_transition('doStep', 'humid2high', 'humid_perfect', conditions = ['isHumidPerfect'], after='doneFan')
        self.fsm.add_transition('doStep', 'humid_perfect', 'humid2high', conditions=['isHumid2high'], after='startFan')
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
    def isHumid2high(self):
        return self.humid >= limits['humidity'][1]
        
    def isHumidPerfect(self):
        return self.humid <= optimal['humidity'][1]
    # END STUDENT CODE
        
    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def startFan(self):
        self.setFan(True)
        print("Turning on the fan to lower humidity")
        
    def doneFan(self):
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
        self.fsm.add_transition('doStep', 'init', 'watering', conditions=['isNextDay', 'notWateredLate'], after=['nextDayWaterLate', 'setAddWater'])
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
        self.states = [self.initial, 'smoist2high', 'smoist_perfect']

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition('enable', self.initial, 'smoist_perfect', after='setInitial')
        
        self.fsm.add_transition('disable', 'smoist2high', self.initial, after='setInitial')
        self.fsm.add_transition('disable', 'smoist_perfect', self.initial, after='setInitial')
          
        self.fsm.add_transition('doStep', 'smoist2high', 'smoist_perfect', conditions = ['isSmoistPerfect'], after='doneFan')
        self.fsm.add_transition('doStep', 'smoist_perfect', 'smoist2high', conditions=['isSmoist2high'], after='startFan')
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
    def isSmoist2high(self):
        return self.smoist >= limits['moisture'][1]
        
    def isSmoistPerfect(self):
        return self.smoist <= optimal['moisture'][1]
    # END STUDENT CODE
        
    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def startFan(self):
        self.setFan(True)
        print("Turning on the fan to lower soil moisture")
        
    def doneFan(self):
        self.setFan(False)
        print("Soil moisture is now perfect!")
    # END STUDENT CODE
            
    def setFan(self, act_state):
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"fan": act_state}))

