import schedule as sched

class BehavioralLayer:

    def __init__(self, sensors, actuators, behaviors):
        self.behaviors = behaviors
      
        for behavior in behaviors:
            behavior.setSensors(sensors)
            behavior.setActuators(actuators)
        self.enabled = []
        self.enabledNames = []
        # Initialize any extra variables here

    def getBehavior(self, name):
        for b in self.behaviors:
            if (b.name == name): return b
        return None

    def isEnabled(self, behavior):
        return behavior in self.enabled

    def startBehavior(self,name):
        # BEGIN STUDENT CODE
        got_behavior = self.getBehavior(name)
        if not self.isEnabled(got_behavior) and got_behavior != None:
        	got_behavior.start()
        	self.enabled.append(got_behavior)
        	self.enabledNames.append(name)
        	
        # END STUDENT CODE
        

    def pauseBehavior(self,name):
        # BEGIN STUDENT CODE
        got_behavior = self.getBehavior(name)
        if self.isEnabled(got_behavior):
        	got_behavior.pause()
        	self.enabled.remove(got_behavior)
        	self.enabledNames.remove(name)
        # END STUDENT CODE
        

    def doStep(self):
        for behavior in self.enabled:
            behavior.doStep()

    def startAll(self):
        for behavior in self.behaviors:
            self.startBehavior(behavior.name)

    #more functions? write them here!

class ExecutiveLayer:

    def __init__(self):
        self.schedule = {}
        self.internalschedule = {}
        self.laststep = -1
        self.monitors = []
        # Initialize any extra variables here
        self.enabled = []
        self.behaviorToEndTime = {}

    def setPlanningLayer(self, planning):
        self.planning = planning

    def setBehavioralLayer(self, behavioral):
        self.behavioral = behavioral

    def setSchedule(self, schedule):
        self.schedule = schedule
        self.internalschedule = schedule

    def requestNewSchedule(self):
        self.planning.requestNewSchedule()

    def getMonitor(self, name):
        for m in self.monitors:
            if (m.name == name): return m
        return None

    def setMonitors(self, sensors,actuator_state, monitorsList):
        self.monitors = monitorsList
        now = sensors.getTime()
        for monitor in self.monitors:
            monitor.setSensors(sensors)
            monitor.setActuatorState(actuator_state)
            monitor.setExecutive(self)
            monitor.last_time = now
            monitor.dt = 0
            monitor.activate()

    def doStep(self, t): #t time in seconds since midnight
        # NOTE: Disable any behaviors that need to be disabled
        #   before enabling any new behaviors
        # BEGIN STUDENT CODE
        
        ct = t/60
        print(ct)
        print(self.behavioral.enabledNames)
        
        
        needs_pausing = set()
        needs_starting = set()
        
        for (behavior_name, intervals) in self.schedule.items():
        	started = False
        	
        	for (start,stop) in intervals:
        		started = (started or (start <= ct and ct < stop))
        	if started and (behavior_name not in self.behavioral.enabledNames):
        		needs_starting.add(behavior_name)
        	elif started == False and (behavior_name in self.behavioral.enabledNames):
        		print(behavior_name + 'needs pausing')
        		needs_pausing.add(behavior_name)
        		
        for beh in needs_pausing:
        	self.behavioral.pauseBehavior(beh)
        for beh2 in needs_starting:
        	self.behavioral.startBehavior(beh2)
        	
        """
        for behavior_name in self.enabled:
        	if self.behaviorToEndTime[behavior_name] <= t:
        		BehavioralLayer.pauseBehavior(self.behavioral, behavior_name)
        		#print('pausing behavior: ' + behavior_name)
        		
        		self.enabled.remove(behavior_name)
        		if len(self.enabled) == 0: 
        			break
        		
        
        for behavior_name in self.schedule:
               tuples = self.internalschedule[behavior_name]
               for thingie in tuples:
               	if thingie[0] <= t and t < thingie[1]:
               		BehavioralLayer.startBehavior(self.behavioral,behavior_name)
       	        	self.enabled.append(behavior_name) 
       	        	self.behaviorToEndTime[behavior_name] = thingie[1]
       	        	break
       	        	
              
       	  """      
       		
      
 
        for monitor in self.monitors:
            monitor.doMonitor()


class PlanningLayer:

    def __init__(self, schedulefile):
        self.schedulefile = schedulefile
        self.usetestfile = False
        self.schedulerequested = True
        self.schedule = {}
        self.laststep = 0

    def setTestingSchedule(self, testschedule):
        self.testschedule = testschedule

    def setExecutive(self, executive):
        self.executive = executive

    def switch_to_test_sched(self):
        self.usetestfile = True
        self.requestNewSchedule()

    def getNewSchedule(self):
        if self.usetestfile:
            self.schedule = self.scheduleFromFile(self.testschedule)
        else:
            self.schedule = self.scheduleFromFile(self.schedulefile)
        self.executive.setSchedule(self.schedule)
        self.schedulerequested = False

    def requestNewSchedule(self):
        self.schedulerequested = True

    def doStep(self, t):
        if self.schedulerequested or self.checkEnded(t):
            self.getNewSchedule()
        self.laststep = (t//60)%(24*60)

    def checkEnded(self, t):
        mins = (t//60)%(24*60)
        if mins < self.laststep: #looped back around
            return True
        return False

    def scheduleFromFile(self, schedulefile):
        schedule = sched.readSchedule(schedulefile)
        return schedule
