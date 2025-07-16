"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_f.py

Title: Lab F - Line Follower

Author: Jaden Tang

Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR should automatically identify the color of a line it sees, then drive on the
center of the line throughout the obstacle course. The RACECAR should also identify
color changes, following colors with higher priority than others. Complete the lines 
of code under the #TODO indicators to complete the lab.

Expected Outcome: When the user runs the script, they are able to control the RACECAR
using the following keys:
- When the right trigger is pressed, the RACECAR moves forward at full speed
- When the left trigger is pressed, the RACECAR, moves backwards at full speed
- The angle of the RACECAR should only be controlled by the center of the line contour
- The RACECAR sees the color RED as the highest priority, then GREEN, then BLUE
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 50

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((300, 0), (rc.camera.get_height(), rc.camera.get_width()))
color = None # the current color that we see for lights
# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!
BLUE = ((80, 100, 100), (125, 255, 255))  # The HSV range for the color blue
GREEN = ((35, 50, 50), (85, 255, 255))  # The HSV range for the color green
RED = ((170, 50, 50), (10, 255, 255))  # The HSV range for the color red
ORANGE = ((10, 100, 100), (20, 255, 255)) # The HSV range for the color orange

# Color priority before seeing red for the first time then not seeing it anymore: Orange > Purple > Red > Green > Blue
# Color priority after seeing red and then not seeing it anymore: Orange > Purple > Green > Blue > Red
COLOR_PRIORITY = (ORANGE, RED, GREEN, BLUE)

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
seen_red = False



########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour():
    global contour_center
    global contour_area
    global color
    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found
        for colors in COLOR_PRIORITY:
            contours = rc_utils.find_contours(image, colors[0], colors[1])
            largestc = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
            if largestc is not None:
                contour_center = rc_utils.get_contour_center(largestc)
                contour_area = rc_utils.get_contour_area(largestc)
                rc_utils.draw_contour(image, largestc)
                rc_utils.draw_circle(image, contour_center)
                color = colors
                break
        # Display the image to the screen
        rc.display.show_color_image(image)

def remap_range(val: float, old_min: float, old_max: float, new_min:float, new_max:float) -> float:
    old_range = old_max - old_min
    new_range = new_max - new_min
    return new_range * (float(val - old_min) / float(old_range)) + new_min

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    rc.drive.set_max_speed(0.5)
    global speed
    global angle
    #rc.drive.set_max_speed(0.125)
    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global seen_red
    global COLOR_PRIORITY

    # Search for contours in the current color image
    update_contour()

    # TODO Part 3: Determine the angle that the RACECAR should receive based on the current 
    # position of the center of line contour on the screen. Hint: The RACECAR should drive in
    # a direction that moves the line back to the center of the screen.

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if color is RED:
        seen_red = True
    if seen_red and color is not RED:
        COLOR_PRIORITY = (ORANGE, GREEN, BLUE, RED)
    
    if contour_center is not None and color is not ORANGE:
        setpoint = rc.camera.get_width()//2
        error = setpoint - contour_center[1] + 180
        angle = max(min(-remap_range(3*error, -320, 320, -1, 1), 1), -1)
        speed = 1
    elif color is ORANGE:
        speed = -1 #wait and brake until it is safe to cross under the moving wall
        
    # Use the triggers to control the car's speed
    '''
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt
    '''

    rc.drive.set_speed_angle(speed, angle)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    """

    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second

    print(COLOR_PRIORITY)
    print(str(color) + " " + str(seen_red))

    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))
    """
    pass



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
