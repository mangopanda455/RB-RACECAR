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
MIN_CONTOUR_AREA = 1500
BLUE = ((80, 100, 100), (125, 255, 255)) 
RED = ((165, 100, 100), (10, 255, 255))
LAST_SEEN = 0
contour_center = None
contour_area = 0
findnext = False
angle = 0
timer = 0
########################################################################################
# Functions                                                                            #
########################################################################################

def remap_range(val: float, old_min: float, old_max: float, new_min:float, new_max:float) -> float:
    old_range = old_max - old_min
    new_range = new_max - new_min
    return new_range * (float(val - old_min) / float(old_range)) + new_min

def update_contour():
    global contour_center
    global contour_area
    global COLOR
    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        contour_blue = rc_utils.find_contours(image, BLUE[0], BLUE[1])
        largest_blue = rc_utils.get_largest_contour(contour_blue, MIN_CONTOUR_AREA)
        contour_red = rc_utils.find_contours(image, RED[0], RED[1])
        largest_red = rc_utils.get_largest_contour(contour_red, MIN_CONTOUR_AREA)
        large_red = 0
        large_blue = 0
        if largest_blue is not None:
            large_blue = rc_utils.get_contour_area(largest_blue)
        if largest_red is not None:
            large_red = rc_utils.get_contour_area(largest_red)
        if ((large_red * 2 >= large_blue and large_blue/2 <= large_red) or (large_blue * 2 >= large_red and large_red/2 <= large_blue)) and large_red > 0 and large_blue > 0:
            #gate format
            contour_center = (0, (rc_utils.get_contour_center(largest_red)[1] + rc_utils.get_contour_center(largest_blue)[1])//2)
        elif LAST_SEEN == RED:
            if largest_blue is not None:
                contour_area = large_blue
                contour_center = rc_utils.get_contour_center(largest_blue)
                rc_utils.draw_contour(image, largest_blue)
                rc_utils.draw_circle(image, contour_center)
                COLOR = BLUE
            else:
                COLOR = None
        elif LAST_SEEN == BLUE:
            if largest_red is not None:
                contour_area = large_red
                contour_center = rc_utils.get_contour_center(largest_red)
                rc_utils.draw_contour(image, largest_red)
                rc_utils.draw_circle(image, contour_center)
                COLOR = RED
            else:
                COLOR = None
        else:
            if largest_red is None and largest_blue is None:
                COLOR = None
                contour_area = 0
                contour_center = None
            elif largest_red is None:
                COLOR = BLUE
                contour_area = large_blue
                contour_center = rc_utils.get_contour_center(largest_blue)
                rc_utils.draw_contour(image, largest_blue)
                rc_utils.draw_circle(image, contour_center)
            elif largest_blue is None:
                COLOR = RED
                contour_area = large_red
                contour_center = rc_utils.get_contour_center(largest_red)
                rc_utils.draw_contour(image, largest_red)
                rc_utils.draw_circle(image, contour_center)
            else:
                if large_blue > large_red:
                    COLOR = BLUE
                    contour_area = large_blue
                    contour_center = rc_utils.get_contour_center(largest_blue)
                    rc_utils.draw_contour(image, largest_blue)
                    rc_utils.draw_circle(image, contour_center)
                else:
                    COLOR = RED
                    contour_area = large_red
                    contour_center = rc_utils.get_contour_center(largest_red)
                    rc_utils.draw_contour(image, largest_red)
                    rc_utils.draw_circle(image, contour_center)
    

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global LAST_SEEN
    global findnext
    global contour_center
    global contour_area
    global angle
    global timer
    angle = 0
    LAST_SEEN = 0
    findnext = True
    contour_center = None
    contour_area = 0
    timer = 0
    rc.drive.set_max_speed(0.3)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global findnext
    global angle
    global timer
    global LAST_SEEN
    if findnext:
        update_contour()
        #print(contour_area)
        if contour_area < 6000 and contour_center is not None:
            setpoint = rc.camera.get_width()//2
            error = setpoint - contour_center[1]
            angle = -remap_range(error, -setpoint, setpoint, -1, 1)
            if angle < -1:
                angle = -1
            if angle > 1:
                angle = 1
        elif COLOR == BLUE:
            LAST_SEEN = BLUE
            angle = -1
            findnext = False
        elif COLOR == RED:
            LAST_SEEN = RED
            angle = 1
            findnext = False
    else:
        scan = rc.lidar.get_samples()
        isdone = False
        if LAST_SEEN == RED:
            _, dist = rc_utils.get_lidar_closest_point(scan, (180, 300))
            if dist < 100:
                isdone = True
        elif LAST_SEEN == BLUE:
            _, dist = rc_utils.get_lidar_closest_point(scan, (60, 180))
            if dist < 100:
                isdone = True
        if isdone:
            findnext = True
            if LAST_SEEN == RED:
                angle = -1
            else:
                angle = 1
        #print(str(angle) + " " +  str(distance) + " " + str(COLOR) + " " + str(findnext) + " " + str(angle) + " " + str(LAST_SEEN) + " " + str(contour_area))
    rc.drive.set_speed_angle(1, angle)

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

