"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: racecar_vision.py

Title: a file

Author: Jaden  Tang

Purpose: things

Expected Outcome: stuff
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np


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
global speed_offset
global angle_offset

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    global speed_offset
    global angle_offset

    speed = 0.0 # The initial speed is at 1.0
    angle = 0.0 # The initial turning angle away from the center is at 0.0
    speed_offset = 0 # The initial speed offset is 0.5
    angle_offset = 0 # The inital angle offset is 1.0

    # This tells the car to begin at a standstill
    rc.drive.stop()

def process_image():
    image = rc.camera.get_color_image()
    image = rc_utils.crop(image, (180, 0), (rc.camera.get_height(), rc.camera.get_width()))
    hsv_lower = (10, 50, 50)
    hsv_upper = (20, 255, 255)
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, hsv_lower, hsv_upper)
    contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    contour_min = 30
    max_contour = contours[0]
    for contour in contours:
        if cv.contourArea(contour) > contour_min:
            if cv.contourArea(contour) > cv.contourArea(max_contour):
                max_contour = contour
    cv.drawContours(image, [max_contour], 0, (0, 255, 0), 3)
    contour_center = rc_utils.get_contour_center(max_contour)
    cv.circle(image, (contour_center[1], contour_center[0]), 6, (0, 255, 255), -1)
    rc.display.show_color_image(image)
    

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle
    global speed_offset
    global angle_offset
    
    # TODO Part 1: Modify the following conditional statement such that when the
    # right trigger is pressed, the RACECAR moves forward at the designated speed.
    # when the left trigger is pressed, the RACECAR moves backward at the designated speed.
    if rc.controller.get_trigger(rc.controller.Trigger.RIGHT) > 0:
        speed = 1
    elif rc.controller.get_trigger(rc.controller.Trigger.LEFT) > 0:
        speed = -1
    else:
        speed = 0
      
    # TODO Part 2: Modify the following conditional statement such that when the
    # value of the left joystick's x-axis is greater than 0, the RACECAR's wheels turn right.
    # When the value of the left joystick's x-axis is less than 0, the RACECAR's wheels turn left.
    (x, y) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    if x > 0.5:
        angle = 1
    elif x < -0.5:
        angle = -1
    else:
        angle = 0

    # TODO Part 3: Write a conditional statement such that when the
    # "A" button is pressed, increase the speed of the RACECAR. When the "B" button is pressed,
    # decrease the speed of the RACECAR. Print the current speed of the RACECAR to the
    # terminal window.
    if rc.controller.was_pressed(rc.controller.Button.A):
        speed_offset += 0.1
        print(f"The current speed of the racecar is {min(max(speed + speed_offset, -1), 1)}")
    elif rc.controller.was_pressed(rc.controller.Button.B):
        speed_offset -= 0.1
        print(f"The current speed of the racecar is {min(max(speed + speed_offset, -1), 1)}")
    # TODO Part 4: Write a conditional statement such that when the
    # "X" button is pressed, increase the turning angle of the RACECAR. When the "Y" button 
    # is pressed, decrease the turning angle of the RACECAR. Print the current turning angle 
    # of the RACECAR to the terminal window.
    if rc.controller.was_pressed(rc.controller.Button.X):
        angle_offset += 0.1
        print(f"The current angle of the racecar is {min(max(angle + angle_offset, -1), 1)}")
    elif rc.controller.was_pressed(rc.controller.Button.Y):
        angle_offset -= 0.1
        print(f"The current angle of the racecar is {min(max(angle + angle_offset, -1), 1)}")
    # Send the speed and angle values to the RACECAR
    rc.drive.set_speed_angle(min(max(speed + speed_offset, -1), 1), min(max(angle + angle_offset, -1), 1))
    process_image()

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
