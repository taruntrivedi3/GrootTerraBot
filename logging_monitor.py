from monitor import *
from datetime import datetime 


class LoggingMonitor(Monitor):

    def __init__(self, period=100):
        super(LoggingMonitor, self).__init__("LoggingMonitor", period)
        # Put any iniitialization code here
        # BEGIN STUDENT CODE
        self.filename = ""
        # END STUDENT CODE
        
    def clock_time(self, unix_stuff):
    	datetime.utcfromtimestamp(unix_stuff).strftime('%Y-%m-%d %H:%M:%S')
    	
    def perceive(self):
        # BEGIN STUDENT CODE
        self.sensordata = self.sensordata 
        self.time = self.sensordata['unix_time']
        #self.converted_time = clock_time(self.time)
        self.actuator_state = self.actuator_state
        # END STUDENT CODE
        pass

    def monitor(self):
        if (self.filename == ""):
            self.filename = f"log{int(self.time)}.txt"
        with open(self.filename, 'a') as f:
            converted_time = self.clock_time(self.time)
            f.write(f"{converted_time}")
            for key in self.sensordata:
                f.write(f", {key}: {self.sensordata[key]}")
            for key in self.actuator_state:
                f.write(f", {key}: {self.actuator_state[key]}")
            f.write("\n")

