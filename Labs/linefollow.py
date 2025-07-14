"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_f.py

Title: Lab F - Line Follower

Author: [PLACEHOLDER] << [Write your name or team name here]

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
import math

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
MIN_CONTOUR_AREA = 10

# A crop window for the floor directly in front of the car

# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!
BLUE = ((80, 100, 100), (130, 255, 255))  # The HSV range for the color blue
GREEN = ((20,50,50),(80,255,255))  # The HSV range for the color green
RED = ((170, 100, 100), (179, 255, 255))
RED2 = ((0, 100, 100), (10, 255, 255))# The HSV range for the color red
ORANGE = ((1, 74, 195), (60, 255, 255))

# Color priority: Red >> Green >> Blue
COLOR_PRIORITY = (ORANGE)

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
extTop = None
lasterror = 0
lasterrors = []
errorstuffs = []
accels = []
velo = []
f = None
MIN_CONTOUR_AREA = 1000  # The minimum area of a contour to be considered valid
CROP_FLOOR = ((250, 0), (rc.camera.get_height(), rc.camera.get_width()))  # The crop window for the floor directly in front of the car


########################################################################################
# Functions
########################################################################################

def clamp(value: float, min_value: float, max_value: float) -> float:
    """
    Clamp the value between min_value and max_value
    """
    if value < min_value:
        return min_value
    elif value > max_value:
        return max_value
    else:
        return value
    
def remap_range(val: float, old_min: float, old_max: float, new_min: float, new_max: float) -> float:
    """
    Remap the value from the old range to the new range
    """
    old_range = old_max - old_min
    new_range = new_max - new_min
    return new_range * (float(val - old_min) / float(old_range)) + new_min

def get_velo():
    global accels
    global velo
    forward_accel = rc.physics.get_linear_acceleration()[2]
    accels.append(forward_accel)
    acc_array = np.array(accels)
    velo = np.cumsum(acc_array) * rc.get_delta_time()


    

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour():
    global contour_center
    global contour_area
    global extTop
    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found

        blue_contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
        green_contours = rc_utils.find_contours(image, GREEN[0], GREEN[1])
        red_contours = rc_utils.find_contours(image, RED[0], RED[1])
        red_contours += rc_utils.find_contours(image, RED2[0], RED2[1])
        
        orange_contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])
    

        # Find the largest contour for each color
        blue_contour = rc_utils.get_largest_contour(blue_contours, MIN_CONTOUR_AREA)
        green_contour = rc_utils.get_largest_contour(green_contours, MIN_CONTOUR_AREA)
        red_contour = rc_utils.get_largest_contour(red_contours, MIN_CONTOUR_AREA)
        orange_contour = rc_utils.get_largest_contour(orange_contours, MIN_CONTOUR_AREA)
        
        # follow contours in the order of color priority
        if blue_contour is not None:
            contour = blue_contour
        elif green_contour is not None:
            contour = green_contour
        elif red_contour is not None:
            contour = red_contour
        elif orange_contour is not None:
            contour = orange_contour
        else:
            contour = None
        # If a valid contour is found, update contour_center and contour_area
        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            
            # extTop = tuple(contour[contour[:, :, 1].argmin()][0])
            # print(extTop)

            
            # Draw the contour and its center on the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
        
        
        # if contour is not None:
        #     contour_center = rc_utils.get_contour_center(contour)
        #     contour_area = rc_utils.get_contour_area(contour)
            
        #     # Draw the contour and its center on the image
        #     rc_utils.draw_contour(image, contour)
        #     rc_utils.draw_circle(image, contour_center)
        # Display the image to the screen
        rc.display.show_color_image(image)

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    global f

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    # rc.drive.set_speed_angle(speed, angle)
    rc.drive.set_max_speed(0.25)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab 2A - Color Image Line Following\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area"
    )
    rc.drive.set_max_speed(0.5)
    f = open("data.txt", 'w')


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global errorstuffs
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global extTop
    global lasterror
    global lasterrors
    

    # Search for contours in the current color image
    update_contour()
    get_velo()

    # TODO Part 3: Determine the angle that the RACECAR should receive based on the current 
    # position of the center of line contour on the screen. Hint: The RACECAR should drive in
    # a direction that moves the line back to the center of the screen.

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if contour_center is not None:
        setpoint = rc.camera.get_width() // 2
        # present_value = extTop[0]
        present_value = contour_center[1]
        kp = -0.003125
        error = setpoint - present_value
        # kp = math.tanh(0.5*error)
        

        kd = -0.001
        # if lasterrors.__len__() > 4:
            
        #     deriv5 = (-1*lasterrors[0] + 8*lasterrors[1] - 8*lasterrors[3] + lasterrors[4]) / (12*rc.get_delta_time())
        #     deriv = deriv5
        deriv = ((error - lasterror) / rc.get_delta_time())
        angle = kp * error + kd * deriv
        # angle += 0.1
        angle = rc_utils.clamp(angle, -1, 1)
        
        # if abs(angle) > 0.4:
        #     speed = 0.3
        # else:
        #     speed = 0.5
        # elif abs(angle) > 0.2:
        #     speed = 0.9
        # elif abs(angle) > 0.05:
        #     speed = 0.95
        # else:
        #     speed = 1
        # if abs(angle) > 0.3:
        #     angle = 1 if angle > 0 else -1
            
        # kpspeed = 0.5
        # setspeed = speed
        # if abs(error) < 2 and contour_area > 5000:
        #     angle = 1
        
        # if error < 0:
        #     angle = 1
        # elif error > 0:
        #     angle = -1
        # else:
        #     angle = 0
        # if contour_area > 10000:
        #     angle = -1
        
        speed = 1
        lasterrors.insert(0, error)
        if len(lasterrors) > 5:
            lasterrors.pop(4)
        lasterror = error
        
        f.write(f"{error} {rc.get_delta_time()}\n")
    else:
        speed = 1
        
    print("current speed: " + str(velo[-1]))

    # Use the triggers to control the car's speed
    # rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    # lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    # speed = rt - lt

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    global angle
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    print(angle)
    # Print a line of ascii text denoting the contour area and x-position
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
            print("".join(s) + " : area = " + str(contour_area) + " speed = " + str(speed))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()

