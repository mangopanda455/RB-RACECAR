"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: template.py << [Modify with your own file name!]

Title: [PLACEHOLDER] << [Modify with your own title]

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: [PLACEHOLDER] << [Write the purpose of the script here]

Expected Outcome: [PLACEHOLDER] << [Write what you expect will happen when you run
the script.]
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
BLUE = ((80, 100, 100), (125, 255, 255))
GREEN = ((35, 100, 100), (85, 255, 255))
RED = ((170, 100, 100), (10, 255, 255))
ORANGE = ((10, 100, 100), (20, 255, 255))
PURPLE = ((125, 100, 100), (165, 255, 255))
COLOR_PRIO = [GREEN, RED, BLUE]
CROP_FLOOR = ((180, 0), (rc.camera.get_height(), rc.camera.get_width()))
MIN_CONTOUR_AREA = 300
contour_center = None
mode = 0
mode_order = [1, 2, 3, 4]
Color = None
angle = 0
seeing = False

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    pass # Remove 'pass' and write your source code for the start() function here

def remap_range(val: float, old_min: float, old_max: float, new_min:float, new_max:float) -> float:
    old_range = old_max - old_min
    new_range = new_max - new_min
    if(old_range != 0):
        return new_range * (float(val - old_min) / float(old_range)) + new_min
    else:
        return 0

def get_markers():
    global seeing
    global mode
    global angle
    image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(image)
    if len(markers) > 0:
        for marker in markers:
            print(f"id: {marker.get_id()}")
        if not seeing:
            #mode+=1
            seeing = True
    else:
        seeing = False

def getmid(input_array):
    a = 0
    b = 0
    for i in range(4):
        a += input_array[i][0]
        b += input_array[i][1]
    return b//4

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  

def update_contour():
    global contour_center
    global contour_area
    global Color
    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        for color in COLOR_PRIO:
            contours = rc_utils.find_contours(image, color[0], color[1])
            largestc = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
            if largestc is not None:
                contour_center = rc_utils.get_contour_center(largestc)
                contour_area = rc_utils.get_contour_area(largestc)
                rc_utils.draw_contour(image, largestc)
                rc_utils.draw_circle(image, contour_center)
                break
        rc.display.show_color_image(image)

def update():
    global angle
    get_markers()
    if mode_order[mode] == 1:
        update_contour()
        setpoint = rc.camera.get_width()//2
        error = setpoint - contour_center[1]
        angle = 5*-remap_range(error, -setpoint, setpoint, -1, 1)
        if angle < -1:
            angle = -1
        if angle > 1:
            angle = 1
    #rc.drive.set_speed_angle(1, angle)


        


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
