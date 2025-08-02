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
sys.path.insert(0, '../library')
import racecar_core
import racecar_utils as rc_utils
import cv2 as cv
import numpy as np
import math
########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
speed = 0
angle = 0
grid = [[0, 0], [0, 10], [7, 0], [3, 3], [1, 5], [1, 0], [1, 10], [1, 15], [1, 23], [1, 8], [2, 5], [2, 0], [2, 10], [2, 15], [2, 23], [2, 8], [3, 5], [3, 0], [3, 10], [3, 15], [3, 23], [3, 8], [4, 5], [4, 0], [4, 10], [4, 15], [4, 23], [5, 5], [5, 0], [5, 10], [5, 15], [5, 23], [5, 8], [6, 5], [6, 0], [6, 10], [6, 15], [6, 23], [6, 8], [0, 1], [0, 2], [0, 5], [0, 6], [0, 7], [0, 11], [0, 12], [0, 16], [0, 17], [0, 18], [0, 20], [0, 21], [0, 22], [7, 3], [7, 5], [7, 6], [7, 7], [7, 10], [7, 13], [7, 16], [7, 17], [7, 18], [7, 20], [7, 21], [7, 22], [4, 1], [4, 2], [4, 5], [4, 6], [4, 7], [4, 10], [4, 11], [4, 12], [4, 15], [4, 20], [4, 21], [4, 22], [4, 23], [1, 3], [2, 3], [5, 1], [6, 2], [7, 3], [1, 13], [2, 13], [3, 13], [5, 11], [6, 12], [7, 13]]
count = 0
previous_angle = 0
GREEN = ((50, 30, 80), (80, 155, 220))
ORANGE = ((7, 29, 176), (15, 205, 220))
COLOR_PRIORITY = (ORANGE, GREEN)
########################################################################################
# Functions
########################################################################################

def start():
    global speed
    global angle
    speed = 1
    angle = 0
    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.1)
 
def update():
    line_follower()

def update_slow():
    update_display()

def line_follower():
    global previous_angle
    kp = 0.003
    speed = 1
    TOP_Y = 290
    BOTTOM_Y = 310

    image = rc.camera.get_color_image()
    image = rc_utils.crop(image, (TOP_Y,0), (BOTTOM_Y,640))
    largest_contour = None
    if image is not None:
        for color in COLOR_PRIORITY:
            contours = rc_utils.find_contours(image, color[0], color[1])
            contour = rc_utils.get_largest_contour(contours, 10)
            if contour is not None:
                if largest_contour is None or rc_utils.get_contour_area(contour) > rc_utils.get_contour_area(largest_contour):
                    largest_contour = contour
    if largest_contour is not None:
        bottom_left_x = 0        
        bottom_left_y = 0
        bottom_right_x = 0
        bottom_right_y = 0
        contour_center = rc_utils.get_contour_center(largest_contour)
        contour_array_size = len(largest_contour)
        top_left_x = largest_contour[0][0][0]
        top_left_y = largest_contour[0][0][1]
        top_right_x = largest_contour[contour_array_size - 1][0][0]
        top_right_y = largest_contour[contour_array_size - 1][0][1]
        for pixel_l in largest_contour:
            if pixel_l[0][1] > bottom_left_y:
                bottom_left_x = pixel_l[0][0]
                bottom_left_y = pixel_l[0][1]
        for pixel_r in largest_contour:
            if pixel_r[0][1] >= bottom_right_y:
                bottom_right_x = pixel_r[0][0]
                bottom_right_y = pixel_r[0][1]
        rc_utils.draw_circle(image, contour_center)
        rc_utils.draw_circle(image, (top_left_y, top_left_x))
        rc_utils.draw_circle(image, (top_right_y, top_right_x))
        rc_utils.draw_circle(image, (bottom_left_y, bottom_left_x))
        rc_utils.draw_circle(image, (bottom_right_y, bottom_right_x))

        top_x = (top_left_x + top_right_x)/2
        bottom_x = (bottom_left_x + bottom_right_x)/2

        error_angle = (top_x - bottom_x)/1.5
        error_position = contour_center[1] - 320
        angle = (error_position + error_angle)*kp

        if(angle > 1):
            angle = 1
        elif(angle < -1):
            angle = -1

    else: 
        angle = previous_angle
    #print(speed)
    rc.drive.set_speed_angle(speed, angle)
    previous_angle = angle
    rc.display.show_color_image(image)
            

def wall_follower():
    RAY_FROM_OFFSET = 50.0
    WINDOW = 3.0
    kp = 0.0042
    speed = 1

    scan = rc.lidar.get_samples()

    right_abs_angle, right_abs_distance = rc_utils.get_lidar_closest_point(scan, (0, 180))
    left_abs_angle, left_abs_distance = rc_utils.get_lidar_closest_point(scan, (181, 360))
    right_offset_distance = rc_utils.get_lidar_average_distance(scan, RAY_FROM_OFFSET, WINDOW)
    left_offset_distance = rc_utils.get_lidar_average_distance(scan, (360 - RAY_FROM_OFFSET), WINDOW)
    front_distance = rc_utils.get_lidar_average_distance(scan, 0, 3)

    if front_distance < 290:
        speed = 0.92
        kp = 0.005 # 0.0044
        print("uh-oh")
    else:
        speed = 1
        kp = 0.00298

    right_angle = right_abs_angle - RAY_FROM_OFFSET
    left_angle = (360 - RAY_FROM_OFFSET) - left_abs_angle

    right_math = (right_offset_distance ** 2) - (right_abs_distance ** 2)
    left_math = (left_offset_distance ** 2) - (left_abs_distance ** 2)

    if right_math < 0:
        right_math = 0
    if left_math < 0:
        left_math = 0
    
    right_parallel_distance = math.sqrt(right_math)
    left_parallel_distance = math.sqrt(left_math)

    error = right_parallel_distance - left_parallel_distance

    angle = error * kp

    if(angle > 1):
        angle = 1
    elif(angle < -1):
        angle = -1

    rc.drive.set_speed_angle(speed, angle)

def cone_slalom():
    image = rc.camera.get_color_image()
    image = rc_utils.crop(image, (210,0), (390,640))
    if image is None:
        red_contour_center = None
        red_contour_area = 0
        blue_contour_center = None
        blue_contour_area = 0
    else:
        red_contours = rc_utils.find_contours(image, (170, 50, 50), (10, 255, 255))
        red_contour = rc_utils.get_largest_contour(red_contours)
        if red_contour is None:
            red_contour_area = 0
            red_contour_center = None
            blue_contour_center = None
        else:
            red_contour_area = rc_utils.get_contour_area(red_contour)
            red_contour_center = rc_utils.get_contour_center(red_contour)


        blue_contours = rc_utils.find_contours(image, (80, 80, 80), (120, 255, 255))
        blue_contour = rc_utils.get_largest_contour(blue_contours)
        if blue_contour is None:
            blue_contour_area = 0
        else:
            blue_contour_area = rc_utils.get_contour_area(blue_contour)
            blue_contour_center = rc_utils.get_contour_center(blue_contour)
    
    if red_contour_area > blue_contour_area and red_contour_area > 3000:
        cone_color = "red"
    elif blue_contour_area > red_contour_area and blue_contour_area > 3000:
        cone_color = "blue"
    else:
        cone_color = "none"

    ####################################################################################
    # LIDAR
    ####################################################################################
    speed = 0.1
    setpoint = 200
    kp = .005
    CONE_WINDOW = 20

    scan = rc.lidar.get_samples()
    closest_cone_angle, closest_cone_distance = rc_utils.get_lidar_closest_point(scan)

    start_index = (closest_cone_angle)*2 - CONE_WINDOW
    last_index = (closest_cone_angle)*2 + CONE_WINDOW

    
    new_scan = scan.pop(closest_cone_angle*2)
    furthest_cone_angle, furthest_cone_distance = rc_utils.get_lidar_closest_point(new_scan)

    print(furthest_cone_angle)

def update_display():
    global count
    matrix = np.zeros((8, 24), dtype=np.uint8)
    for i in grid:
        if(i[1] + count) % 29 < 24:
            matrix[i[0], (i[1]+count)%29] = 1
    rc.display.set_matrix(matrix)
    count -= 1

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.set_update_slow_time(0.8)
    rc.go()
