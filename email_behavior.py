from behavior import *
from transitions import Machine
import time
from send_email import send
from pathlib import Path

'''
The behavior should send an email that includes the team name and TerraBot
number, the date and time, the current sensor and actuator readings, and
the most recent image taken
'''

to_emails = 'taward@andrew.cmu.edu, ttrivedi@andrew.cmu.edu, yifansun@andrew.cmu.edu'
from_email = 'TerraBot9@outlook.com'
from_pass = 'Admoni467'
group_name = 'I am Groot / TB9 '

class Email(Behavior):
    def __init__(self):
        super(Email, self).__init__("EmailBehavior")
        # Your code here
	# Initialize the FSM and add transitions
        # BEGIN STUDENT CODE
        self.initial = 'halt'
        self.states = [self.initial, 'send']
        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)
        self.fsm.add_transition('enable', 'halt', 'send', before='send_email')
        self.fsm.add_transition('disable', 'send', 'halt')
        # END STUDENT CODE

    # Add the condition and action functions
    #  Remember: if statements only in the condition functions;
    #            modify state information only in the action functions
    # BEGIN STUDENT CODE
    def send_email(self):
        current_path = Path(__file__).resolve()
        pictures_directory = ((current_path.parent).parent).parent / 'pictures'

        date_string = time.strftime('%Y-%m-%d', time.localtime(self.time)) # chat gpt
        text = f"""\
        TerraBot9 sensor values:
        - time: {self.time}
        - midnight time: {self.mtime}
        - light: {self.light}
        - temp: {self.temp}
        - humid: {self.humid}
        - water level: {self.water_level}
        - weight: {self.weight}
        - soil moisture: {self.smoist}
        
        """
    
        # get images
        files = [f.name for f in pictures_directory.iterdir() if f.is_file()] # chat gpt
        images = []
        most_recent = ""
        if files != []:
            most_recent = max(files, key=lambda x: int(x.lower().split('pic')[1].split('.jpg')[0])) # chat gpt
            images = [pictures_directory / most_recent]
        else:
            text = text + "no images in TerraBot/pictures"
        send(from_email, from_pass, to_emails, group_name + date_string, text, most_recent, images, True)
    # END STUDENT CODE

    def perceive(self):
        self.time = self.sensordata['unix_time']
        # Add any sensor data variables you need for the behavior
        # BEGIN STUDENT CODE
        self.mtime = self.sensordata["midnight_time"]
        self.light = self.sensordata["light"]
        self.temp = self.sensordata["temp"]
        self.humid = self.sensordata["humid"]
        self.water_level = self.sensordata["level"]
        self.weight = self.sensordata["weight"]
        self.smoist = self.sensordata["smoist"]
        # END STUDENT CODE

    def act(self):
        self.trigger("doStep")

