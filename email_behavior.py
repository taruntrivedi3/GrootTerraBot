from behavior import *
from transitions import Machine
import time
from send_email import send
from pathlib import Path
from terrabot_utils import clock_time
from vision import plantHealth, foliageImages
from cv_utils import readImage, writeImage

'''
The behavior should send an email that includes the team name and TerraBot
number, the date and time, the current sensor and actuator readings, and
the most recent image taken
'''

#to_emails = 'taward@andrew.cmu.edu'
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

        self.last_plant_health = None
        self.greenery_y = None
        self.health_msg_y = None

        self.water_level_y = None

        first_day = True

        with open('water.txt', 'w') as file:
            file.write("0.0")
        # END STUDENT CODE

    # Add the condition and action functions
    #  Remember: if statements only in the condition functions;
    #            modify state information only in the action functions
    # BEGIN STUDENT CODE
    def send_email(self):
        print("PREPARING EMAIL")
        current_path = Path(__file__).resolve()
        pictures_directory = ((current_path.parent).parent).parent / 'pictures'
        modified_pictures_directory = ((current_path.parent).parent).parent / 'modified_pictures'

        # Calculating how much water was pumped in over the last day
        try:
            with open('water.txt', 'r') as file:
                daily_total = float(file.read())
            if daily_total > 0.0:
                if self.water_level_y == None:
                    water_msg = f"""The agent has been watered {daily_total}ml of water over the past 24 hours."""
                else:
                    water_diff = self.water_level_y - self.water_level
                    water_msg = f"""The agent has been watered in the past 24 hours. {round(water_diff, 2)}mm of water from the reservoir was used. This accounts for {daily_total}ml of water."""
            else:
                water_msg = "The agent has not been watered over the past 24 hours."

        except:
            water_msg = "We are unable to access the water level right now."

        self.water_level_y = self.water_level
        with open('water.txt', 'w') as file:
            file.write("0.0")

        # Get yesterday's insolation from text file
        try:
            with open('insolation.txt', 'r') as file:
                insolation = file.read()
            print(insolation)
            insolation_msg = f"""Yesterday there was a total insolation of {round(float(insolation.strip()), 2)}."""

        except:
            insolation_msg = "We are not able to currently access yesterday's total insolation."

        first_day = False

        # Getting plant health and comparing to yesterdays
        greenery = 0
        health_msg = ""
        if self.health_msg_y == None:
            yesterday_msg = "had not been started yet"
        else:
            yesterday_msg = f"""was doing {self.health_msg_y} with a greenery percent of {round(self.greenery_y, 2)}%"""

        date_string = time.strftime('%m-%d-%Y', time.localtime(self.time)) # chat gpt
    
        # Get most recent image, apply filters to them and save them, then get new image
        files = [f.name for f in pictures_directory.iterdir() if f.is_file()] # chat gpt
        img_paths = []
        most_recent = ""
        if files != []:
            most_recent = max(files, key=lambda x: int(x.lower().split('pic')[1].split('.jpg')[0])) # chat gpt
            img_paths = [pictures_directory / most_recent]
        else:
            text = text + "no images in TerraBot/pictures"

        modified_paths = []

        for image in img_paths:

            # get plant health
            greenery, health_msg = plantHealth(readImage(str(image)))

            # get masks
            masked, outline, h = foliageImages(readImage(str(image)))
            masked_name = "masked_" + most_recent
            outlined_name = "outlined_" + most_recent
            writeImage(str((modified_pictures_directory / masked_name)), masked)
            writeImage(str((modified_pictures_directory / outlined_name)), outline)
            modified_paths = [(modified_pictures_directory / masked_name, masked_name), (modified_pictures_directory / outlined_name, outlined_name)]

        images = []
        for image, name in modified_paths:
            with open(image, 'rb') as f:
                image_data = f.read()
            images.append((name, image_data))

        # write message
        text = f"""\
        TERRABOT9 SUMMARY:
            It is currently {date_string}. It is currently day {clock_time(self.time)} of this grow period. These are the current sensor values for TerraBot9:
                -> The temperature in the greenhouse is {round(self.temp, 2)} degrees Celsius.
                -> The light intensity is {round(self.light, 2)}/1000.
                -> The humidity is {round(self.humid, 2)}%.
                -> The water level in the resevoir is {round(self.water_level, 2)}mm.
                -> The weight of soil and plants is {round(self.weight, 2)}g.
                -> The soil moisture is {round(self.smoist, 2)}.

        WATERING:
            {water_msg}

        INSOLATION:
            {insolation_msg}

        PLANT HEALTH:
            Our most recent image has a foliage percent of {round(greenery, 2)}%. We have determined the plant is doing {health_msg}, yesterday it {yesterday_msg}.

        """

        self.greenery_y = greenery
        self.health_msg_y = health_msg

        # send email
        send(from_email, from_pass, to_emails, group_name + date_string, text, images, True)
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

