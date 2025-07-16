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
MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((300, 0), (rc.camera.get_height(), rc.camera.get_width()))

# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!
BLUE = ((80, 100, 100), (120, 255, 255))  # The HSV range for the color blue
GREEN = ((35, 50, 50), (75, 255, 255))
RED = ((0, 100, 100),(20, 255, 255))

# Color priority: Red >> Green >> Blue
##COLOR_PRIORITY = (RED, GREEN, BLUE)

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
found_contour = None


########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour(color):
    global contour_center
    global contour_area
    global found_contour

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found

        contours = rc_utils.find_contours(image, color[0], color[1])

        # Display the image to the screen
        rc.display.show_color_image(image)

        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
            found_contour = color
        else:
            found_contour = None
        
     

    rc.display.show_color_image(image)

        

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

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

    
    
    update_contour(RED)
    if found_contour == RED:
        print("Red Contour Found")
    #update_contour(GREEN)


    if found_contour is not RED:
        
        update_contour(GREEN)
        if found_contour == GREEN:
            print("Green Contour Found")

    if found_contour is not GREEN:
        update_contour(BLUE)
        #print("Blue Not Found")
        if found_contour == BLUE:
            print("Blue Contour Found")
    if found_contour is not BLUE and not GREEN:
        print("No Contour Found")
    
    

    #if found_contour == RED:
        #print("Red Found")
    #elif found_contour == BLUE:
        #print("Blue Found")
    #elif found_contour == GREEN:
        #print("Green Found")
    #else:
        #print("No contour Found")

    #if red is not None:


    #try:
        #update_contour(RED)
        #red = True
        #print("red found")
        
    #except:
        #print("No Red Line Found")
        #pass
    

    #if red == False:
        #try:
            #update_contour(BLUE)
            #blue = True
           
        #except:
            
            #pass

    #if blue == False:
        #try:
            #update_contour(GREEN)
            #green = True
            
        #except:
            #print("No Lines Found")

    

   

    ## clamp values
    if contour_center is not None:
        setpoint = rc.camera.get_width() // 2
        error = setpoint - contour_center[1]

        if error < 0:
            angle = 1
        elif error > 0:
            angle = -1
        else:
            angle = 0

        if abs(error) < 5 and contour_area > 5000:
            angle = 1

    # Search for contours in the current color image
    

    # TODO Part 3: Determine the angle that the RACECAR should receive based on the current 
    # position of the center of line contour on the screen. Hint: The RACECAR should drive in
    # a direction that moves the line back to the center of the screen.

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle

    ## figure out the angle to turn at
    if contour_center is not None:
        setpoint = rc.camera.get_width() // 2 # cemter of tje screen

        current_value = contour_center[1]

        kp = -0.003125

        error = setpoint - current_value

        angle = kp * error

        rc_utils.clamp(angle, -1, 1)
        

    speed = 1 - abs(angle)

    if speed < 0.45:
        speed = 0.45

    if contour_center is None:
        speed = -1
        angle = -angle

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
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
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
            print("".join(s) + " : area = " + str(contour_area))

def clamp(value: float, min: float, max: float) -> float:
    if value < min:
        return min
    if value > max:
        return max
    else:
        return value
    



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()

