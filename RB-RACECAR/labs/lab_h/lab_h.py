"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_h.py

Title: Cone Slalom

Author: Jaden Tang

Purpose: Rules of the Road: To navigate through the cone slalom course, program your RACECAR to follow two rules:

Drive on the right side of RED cones
Drive on the left side of BLUE cones

Expected Outcome: 
Rules of the Road: To navigate through the cone slalom course, program your RACECAR to follow two rules:

Drive on the right side of RED cones
Drive on the left side of BLUE cones
"""

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
CROP_FLOOR = ((180, 0), (rc.camera.get_height(), rc.camera.get_width()))
MIN_CONTOUR_AREA = 275
BLUE = ((80, 100, 100), (125, 255, 255))  # The HSV range for the color blue
RED = ((165, 100, 100), (10, 255, 255))  # The HSV range for the color red
COLOR = None
LAST_SEEN = 0
change = 0
angle = 0

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global change
    global angle
    global LAST_SEEN
    global COLOR
    global timer
    global contour_area
    global contour_center
    contour_area = 0
    contour_center = None
    COLOR = None
    change = 0
    angle = 0
    LAST_SEEN = None
    timer = 0
    rc.drive.set_max_speed(0.15)
    print("Cone Slalom Time!")

def remap_range(val: float, old_min: float, old_max: float, new_min:float, new_max:float) -> float:
    old_range = old_max - old_min
    new_range = new_max - new_min
    return new_range * (float(val - old_min) / float(old_range)) + new_min

def update_contour():
    global distance_front
    global contour_center
    global contour_area
    global COLOR
    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found
        if LAST_SEEN == RED:
            contour_blue = rc_utils.find_contours(image, BLUE[0], BLUE[1])
            largest_blue = rc_utils.get_largest_contour(contour_blue, MIN_CONTOUR_AREA)
            if largest_blue is not None:
                contour_area = rc_utils.get_contour_area(largest_blue)
                contour_center = rc_utils.get_contour_center(largest_blue)
                rc_utils.draw_contour(image, largest_blue)
                rc_utils.draw_circle(image, contour_center)
                COLOR = BLUE
            else:
                COLOR = None
        elif LAST_SEEN == BLUE:
            contour_blue = rc_utils.find_contours(image, RED[0], RED[1])
            largest_blue = rc_utils.get_largest_contour(contour_blue, MIN_CONTOUR_AREA)
            if largest_blue is not None:
                contour_area = rc_utils.get_contour_area(largest_blue)
                contour_center = rc_utils.get_contour_center(largest_blue)
                rc_utils.draw_contour(image, largest_blue)
                rc_utils.draw_circle(image, contour_center)
                COLOR = RED
            else:
                COLOR = None
        else:
            contour_red = rc_utils.find_contours(image, RED[0], RED[1])
            contour_blue = rc_utils.find_contours(image, BLUE[0], BLUE[1])
            largest_red = rc_utils.get_largest_contour(contour_red, MIN_CONTOUR_AREA)
            largest_blue = rc_utils.get_largest_contour(contour_blue, MIN_CONTOUR_AREA)
            if largest_red is None and largest_blue is None:
                COLOR = None
                contour_area = 0
                contour_center = None
            elif largest_red is None:
                COLOR = BLUE
                contour_area = rc_utils.get_contour_area(largest_blue)
                contour_center = rc_utils.get_contour_center(largest_blue)
                rc_utils.draw_contour(image, largest_blue)
                rc_utils.draw_circle(image, contour_center)
            elif largest_blue is None:
                COLOR = RED
                contour_area = rc_utils.get_contour_area(largest_red)
                contour_center = rc_utils.get_contour_center(largest_red)
                rc_utils.draw_contour(image, largest_red)
                rc_utils.draw_circle(image, contour_center)
            else:
                if rc_utils.get_contour_area(largest_blue) > rc_utils.get_contour_area(largest_red):
                    COLOR = BLUE
                    contour_area = rc_utils.get_contour_area(largest_blue)
                    contour_center = rc_utils.get_contour_center(largest_blue)
                    rc_utils.draw_contour(image, largest_blue)
                    rc_utils.draw_circle(image, contour_center)
                else:
                    COLOR = RED
                    contour_area = rc_utils.get_contour_area(largest_red)
                    contour_center = rc_utils.get_contour_center(largest_red)
                    rc_utils.draw_contour(image, largest_red)
                    rc_utils.draw_circle(image, contour_center)
        rc.display.show_color_image(image)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global change
    global angle
    global LAST_SEEN
    global timer
    print(str(contour_area) + " " + str(angle) + " " + str(timer) + " " + str(COLOR))
    if timer <= 0:
        update_contour()
    if contour_area < 15000 and contour_center is not None and timer <= 0:
        setpoint = rc.camera.get_width()//2
        error = setpoint - contour_center[1]
        angle = -remap_range(error, -setpoint, setpoint, -1, 1)
    elif COLOR == BLUE and timer <= 0:
        LAST_SEEN = BLUE
        angle = -1
        change = 1
        timer = 1
    elif COLOR == RED and timer <= 0:
        LAST_SEEN = RED
        angle = 1
        change = -1
        timer = 1
    else:
        if timer > 0:
            timer -= rc.get_delta_time()
        else:
            if angle != change:
                angle = change
            else:
                timer = -1
    rc.drive.set_speed_angle(1, angle)
    

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    print(contour_area)
    pass # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
