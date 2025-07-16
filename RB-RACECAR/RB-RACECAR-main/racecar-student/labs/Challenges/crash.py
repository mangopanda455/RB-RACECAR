
"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs"""

########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
global speed
global angle


########################################################################################
# Functions
########################################################################################

def find_distance():
    scan = rc.lidar.get_samples()

    _ , distance = rc_utils.get_lidar_closest_point(scan, (355, 5))  
    distance = distance - 19

    return distance

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    

    speed = 0.0 # The initial speed is at 1.0
    angle = 0.0 # The initial turning angle away from the center is at 0.0
    

    # This tells the car to begin at a standstill
    rc.drive.stop()

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle
    
    if rc.controller.get_trigger(rc.controller.Trigger.RIGHT) > 0:
        speed = 1
    elif rc.controller.get_trigger(rc.controller.Trigger.LEFT) > 0:
        speed = -1
    else:
        speed = 0
      
    (x, _) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    angle = x
    print(x)

    distance = find_distance()

    if distance < 10:
        print(distance)
        speed = -1
    if distance > 20:
        speed = 0
    elif distance > 10:
        speed = 0.5
    if distance < 10:
        speed = -1
    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()


