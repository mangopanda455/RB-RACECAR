"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_e.py

Title: Lab E - Stoplight Challenge

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: Write a script to enable autonomous behavior from the RACECAR. When
the RACECAR sees a stoplight object (colored cube in the simulator), respond accordingly
by going straight, turning right, turning left, or stopping. Append instructions to the
queue depending on whether the position of the RACECAR relative to the stoplight reaches
a certain threshold, and be able to respond to traffic lights at consecutive intersections. 

Expected Outcome: When the user runs the script, the RACECAR should control itself using
the following constraints:
- When the RACECAR sees a BLUE traffic light, make a right turn at the intersection
- When the RACECAR sees an ORANGE traffic light, make a left turn at the intersection
- When the RACECAR sees a GREEN traffic light, go straight
- When the RACECAR sees a RED traffic light, stop moving,
- When the RACECAR sees any other traffic light colors, stop moving.

Considerations: Since the user is not controlling the RACECAR, be sure to consider the
following scenarios:
- What should the RACECAR do if it sees two traffic lights, one at the current intersection
and the other at the intersection behind it?
- What should be the constraint for adding the instructions to the queue? Traffic light position,
traffic light area, or both?
- How often should the instruction-adding function calls be? Once, twice, or 60 times a second?

Environment: Test your code using the level "Neo Labs > Lab 3: Stoplight Challenge".
By default, the traffic lights should direct you in a counterclockwise circle around the course.
For testing purposes, you may change the color of the traffic light by first left-clicking to 
select and then right clicking on the light to scroll through available colors.
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
# The smallest contour we will recognize as a valid contour (Adjust threshold!)
MIN_CONTOUR_AREA = 1
CROP_FLOOR = ((100, 0), (rc.camera.get_height(), rc.camera.get_width()))
# TODO Part 1: Determine the HSV color threshold pairs for ORANGE, GREEN, RED, YELLOW, and PURPLE
# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((80, 100, 100), (125, 255, 255))  # The HSV range for the color blue
GREEN = ((30, 100, 100), (80, 255, 255))  # The HSV range for the color green
RED = ((165, 100, 100), (10, 255, 255))  # The HSV range for the color red
ORANGE = ((10, 100, 100), (20, 255, 255)) # The HSV range for the color orange
YELLOW = ((20, 100, 100), (30, 255, 255)) # The HSV range for the color yellow
PURPLE = ((125, 100, 100), (165, 255, 255)) # The HSV range for the color purple
COLORS = [BLUE, GREEN, RED, ORANGE, YELLOW, PURPLE]
# >> Variables
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
speed = 0
angle = 0
queue = [] # The queue of instructions
stoplight_color = None # The current color of the stoplight
hasTurnedRight = False
hasTurnedLeft = False
########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour():
    global contour_center
    global contour_area
    global stoplight_color
    image = rc.camera.get_color_image()
    contour_center = None
    contour_area = 0
    stoplight_color = None
    stoplight_color = "NONE"
    if image is not None:
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found
        for color in COLORS:
            contours = rc_utils.find_contours(image, color[0], color[1])
            largestc = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
            if largestc is not None:
                if(rc_utils.get_contour_area(largestc) > contour_area):
                    contour_area = rc_utils.get_contour_area(largestc)
                    contour_center = rc_utils.get_contour_center(largestc)
                    stoplight_color = color
                    
        # TODO Part 3: Repeat the search for all potential traffic light colors,
        # then select the correct color of traffic light detected.
        # Display the image to the screen
    if largestc is not None:
        rc_utils.draw_contour(image, largestc)
        rc_utils.draw_circle(image, contour_center)
    rc.display.show_color_image(image)

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global queue, hasTurnedLeft, hasTurnedRight
    hasTurnedRight = False
    hasTurnedLeft = False
    queue.clear()
    # Set initial driving speed and angle
    rc.drive.set_speed_angle(0,0)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message (You may edit this to be more informative!)
    print(
        ">> Lab 3 - Stoplight Challenge\n"
        "\n"
        "Controls:\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area"
    )

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global queue
    #print(str(queue) + " " + str(stoplight_color))
    update_contour()

    # TODO Part 2: Complete the conditional tree with the given constraints.
    if stoplight_color == BLUE:
        turnRight()
        pass
    elif stoplight_color == GREEN:
        goStraight()
        pass
    elif stoplight_color == ORANGE:
        turnLeft()
        pass
    elif stoplight_color == RED:
        stopNow()

    # TODO Part 3: Implement a way to execute instructions from the queue once they have been placed
    # by the traffic light detector logic (Hint: Lab 2)
    if len(queue) > 0:
        speed = queue[0][1]
        angle = queue[0][2]
        queue[0][0] -= rc.get_delta_time()
        if queue[0][0] <= 0:
            queue.pop(0)
    else:
        speed = 0
        angle = 0
    # Send speed and angle commands to the RACECAR
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
def update_slow():
    pass

# [FUNCTION] Appends the correct instructions to make a 90 degree right turn to the queue
def turnRight():
    global queue, hasTurnedRight
    if len(queue) == 0:
        queue.append([2, 1, 0])
        if(hasTurnedRight):
            queue.append([1.25, 1, 1])
        else:
            queue.append([1.35, 1, 1])
            hasTurnedRight = True
    # TODO Part 4: Complete the rest of this function with the instructions to make a right turn

# [FUNCTION] Appends the correct instructions to make a 90 degree left turn to the queue
def turnLeft():
    global queue, hasTurnedLeft
    if len(queue) == 0:
        queue.append([2, 1, 0])
        if(hasTurnedLeft):
            queue.append([1.25, 1, -1])
        else:
            queue.append([1.15, 1, -1])
            hasTurnedLeft = True
    
    # TODO Part 5: Complete the rest of this function with the instructions to make a left turn

# [FUNCTION] Appends the correct instructions to go straight through the intersectionto the queue
def goStraight():
    global queue
    if len(queue) == 0:
        queue.append([1.75, 1, 0])
    # TODO Part 6: Complete the rest of this function with the instructions to make a left turn

# [FUNCTION] Clears the queue to stop all actions
def stopNow():
    global queue
    #queue.clear()

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()