from behavior import *
from transitions import Machine

'''
The behavior should ping once every 2-3 minutes
'''
class Ping(Behavior):

    def __init__(self):
        super(Ping, self).__init__("PingBehavior")
        self.initial = 'initial'
        # STUDENT CODE: Modify these lines to add all your FSM states
        self.states = [self.initial,'initialWaiting']

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        self.fsm.add_transition('enable',self.initial,'initialWaiting',after='setInitial')
        self.fsm.add_transition('doStep','initialWaiting','initialWaiting',conditions='timeCheck',after='afterTimeCheck')
        self.fsm.add_transition('disable','initialWaiting',self.initial,after='setInitial')
        # BEGIN STUDENT CODE
        
        # END STUDENT CODE

    def setInitial(self):
        self.last_ping = -10000

    def perceive(self):
        self.time = self.sensordata["unix_time"]

    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger('doStep')

    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def timeCheck(self):
    	return self.time - self.last_ping >= 120
            
    # END STUDENT CODE

    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def afterTimeCheck(self):
    	self.actuators.doActions((self.name, self.time, {"ping":True})); self.last_ping = self.time
    # END STUDENT CODE

    def ping(self):
        self.actuators.doActions((self.name, self.time, {"ping":True}))
        self.last_ping = self.time

