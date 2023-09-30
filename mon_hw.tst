# Wait a minute before starting, to give agent a chance to initialize
DELAY FOR 60

# any failures could be due to sensor problems or otherwise specified

# TEST 1: Make sure LEDS are set to 0 between 10pm and 8am (5 min buffer)
WHENEVER 1-22:05:00
    ENSURE not led UNTIL 2-07:55:00

# TEST 2: Make sure when LowerHumidBehavior is enabled and humidity is over limit fan is on
WHENEVER enabled("LowerHumidBehavior") and (humidity[0] > 90 and humidity[1] > 90) and mtime < 44000 and mtime > 20000
    WAIT fan FOR 300

# TEST 3: Make sure the bot is watered when raise moist is on and moisture is too low by checking weight change
# this test could fail if the plant is not watered if there is not enough water in tank or plant has been watered past daily limit
WHENEVER enabled("RaiseMoistBehavior") and (smoist[0] < 550 and smoist[1] < 550)
    SET start_total_weight = weight[0] + weight[1]
    WAIT FOR 1800
    ENSURE (weight[0] + weight[1] > start_total_weight)

# TEST 4: Check multiple behaviors are enabled at a given time
WHENEVER 1-21:35:00
    ENSURE enabled("LightBehavior") and enabled("LowerTempBehavior")
    ENSURE not enabled("RaiseTempBehavior") and not enabled("RaiseMoistBehavior") and not enabled("LowerMoistBehavior")

# TEST 5: Temperature raises when raisetemp is on and below limits
WHENEVER enabled("RaiseTemp") and (temperature[0] < 25 and temperature[1] < 25)
    SET start_temp = temperature[0] + temperature[1]
    WAIT FOR 600
    ENSURE start_temp < temperature[0] + temperature[1]
    
# TEST 6: It should never be not humid enough except at the very beginning
WHENEVER humidity[0] < 60 and humidity[1] < 60 and mtime > 3600
    ENSURE False

# TEST 7: When soil moisture is above limit make and LowerMoist behavior is on fan should be on and moisture should lower
WHENEVER (smoist[0] > 650 and smoist[1] > 650) and enabled("LowerMoist")
    WAIT 60
    SET starting_moist = smoist[0] + smoist[1]
    ENSURE fan
    WAIT 360
    ENSURE starting_moist < smoist[0] + smoist[1]

# TEST 8: Check most sensor values are similar
# failure would imply a sensor is broken
WHENEVER 1-12:00:00
    ENSURE (light[0] < light[1] + 10)  and (light[0] > light[1] - 10)
    ENSURE (smoist[0] < smoist[1] + 10) and (smoist[0] > smoist[1] - 10)
    ENSURE (temperature[0] < temperature[1] + 2) and (temperature[0] > temperature[1] - 2)
    ENSURE (humidity[0] < humidity[1] + 5) and (humidity[0] > humidity[1] - 5)


