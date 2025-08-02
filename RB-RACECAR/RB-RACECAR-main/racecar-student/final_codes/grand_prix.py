########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

## MISC IMPORTS ###
import random
import numpy as np
import cv2 as cv
import math

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
speed = 0
angle = 0

## State Machine
LEVELS = {1: "waiting", 2: "wall_follower", 3: "line_follower", 4: "cone_salom", 5: "fork", 6: "elevator"}
current_level = LEVELS[0]

## Contours
# crops the top 100 poixels of the image
CROP_FLOOR = ((100, 0), (rc.camera.get_height(), rc.camera.get_width()))
MIN_CONTOUR_AREA = 200

## AR Tags
image = rc.camera.get_color_image()
markers = rc_utils.get_ar_markers(image)
AR_id = None

## Cone Salom

CROP_FLOOR_CONE = ((180, 0), (rc.camera.get_height(), rc.camera.get_width()))
MIN_CONTOUR_AREA_CONE = 700
CONE_1 = ((1, 103, 149), (20, 239, 251))
CONE_2 = ((39, 52, 136), (67, 224, 255))
COLOR_CONE = None
LAST_SEEN = 0
findnext = True

contour_area_cone = 0
contour_center_cone = None


## Line Follower

previous_angle = 0
error_position = 0

## Gate

past_id = 0
counter = 0
obj = None
open = False

########################################################################################
# Functions
########################################################################################

#--------------------------------------------------------------------------------------#
# MISC FUNCTIONS
#--------------------------------------------------------------------------------------#

########## DOT MATRIX ##########

def name_display():
    global num
    global matrix
    
    # define the pixels to light up
    row_1 = [0, 1, 2, 5, 6, 7, 10, 11, 12, 16, 17, 18, 20, 21, 22, 23]
    row_2 = [0, 3, 5, 8, 10, 13, 15, 23]
    row_3 = [0, 3, 5, 10, 13, 15, 23, 8]
    row_4 = [0, 1, 2, 5, 6, 7, 10, 11, 12, 15, 22, 23]
    row_5 = [0, 3, 5, 8, 13, 15, 23, 10]
    row_6 = [0, 3, 5, 13, 15, 23, 8, 10]
    row_7 = [0, 3, 5, 15, 23, 8, 13, 10]
    row_8 = [0, 3, 5, 6, 7, 10, 13, 16, 17, 18, 20, 21, 22, 23]

    # organize it into rows 
    rows = [row_1, row_2, row_3, row_4, row_5, row_6, row_7, row_8]

    # display all the pixels in the row 
    for pixel in rows[num]:
        matrix[num, pixel] = 1

    # decrease the counter for the row to display
    if num > 0:
        num -= 1

    rc.display.set_matrix(matrix)

def speed_display():
    global status
    global speed
    global matrix

    if status == "Speed":
        #zero out the dot matrix
        matrix = np.zeros((8, 24), dtype=np.uint8)
        # for all 8 rows, set the first pixel to random
        for i in range(8):
            if speed == 0:
                int = random.randint(0, 1)
                matrix[i-1, 0] = int
        # pretty much this just checks the speed and sets the lights depending on the speed.
        # im not commending all 700 lines of code.
        # it makes a really cool random effect
        # trust

        if abs(speed) > 0.6 and abs(speed) < 0.62:
            for i in range(8):
                #col 1
                int = random.randint(0, 1)
                matrix[i-1, 1] = int
        elif abs(speed) > 0.62 and abs(speed) < 0.64:
            for i in range(8):
                #col 2
                int = random.randint(0, 1)
                matrix[i-1, 2] = int
                int = random.randint(0, 1)
                matrix[i-1, 1] = int

        elif abs(speed) > 0.64 and abs(speed) < 0.66:
            for i in range(8):
                #col 3
                int = random.randint(0, 1)
                matrix[i-1, 3] = int
                int = random.randint(0, 1)
                matrix[i-1, 2] = int
                matrix[i-1, 1] = int

        elif abs(speed) > 0.66 and abs(speed) < 0.68:
            for i in range(8):
                #col 4
                int = random.randint(0, 1)
                matrix[i-1, 4] = int
                int = random.randint(0, 1)
                matrix[i-1, 3] = int
                matrix[i-1, 2] = int
                matrix[i-1, 1] = 1

        elif abs(speed) > 0.68 and abs(speed) < 0.7:
            for i in range(8):
                #col 5
                int = random.randint(0, 1)
                matrix[i-1, 5] = int
                int = random.randint(0, 1)
                matrix[i-1, 4] = int
                matrix[i-1, 3] = int
                for x in range(2):
                    matrix[i-1, x+1] = 1
                

        elif abs(speed) > 0.7 and abs(speed) < 0.72:
            for i in range(8):
                #col 6
                int = random.randint(0, 1)
                matrix[i-1, 6] = int
                int = random.randint(0, 1)
                matrix[i-1, 5] = int
                matrix[i-1, 4] = int
                for x in range(3):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.72 and abs(speed) < 0.74:
            for i in range(8):
                #col 7
                int = random.randint(0, 1)
                matrix[i-1, 7] = int
                int = random.randint(0, 1)
                matrix[i-1, 6] = int
                matrix[i-1, 5] = int
                for x in range(4):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.74 and abs(speed) < 0.76:
            for i in range(8):
                #col 8
                int = random.randint(0, 1)
                matrix[i-1, 8] = int
                int = random.randint(0, 1)
                matrix[i-1, 7] = int
                matrix[i-1, 6] = int
                for x in range(5):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.76 and abs(speed) < 0.77:
            for i in range(8):
                #col 9
                int = random.randint(0, 1)
                matrix[i-1, 9] = int
                int = random.randint(0, 1)

                matrix[i-1, 8] = int
                matrix[i-1, 7] = int
                for x in range(6):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.77 and abs(speed) < 0.78:
            for i in range(8):
                #col 10
                int = random.randint(0, 1)
                matrix[i-1, 10] = int
                int = random.randint(0, 1)

                matrix[i-1, 9] = int
                matrix[i-1, 8] = int
                for x in range(7):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.78 and abs(speed) < 0.79:
            for i in range(8):
                #col 11
                int = random.randint(0, 1)
                matrix[i-1, 11] = int
                int = random.randint(0, 1)

                matrix[i-1, 10] = int
                matrix[i-1, 9] = int
                for x in range(8):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.79 and abs(speed) < 0.8:
            for i in range(8):
                #col 12
                int = random.randint(0, 1)
                matrix[i-1, 12] = int
                int = random.randint(0, 1)

                matrix[i-1, 11] = int
                matrix[i-1, 10] = int
                for x in range(9):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.8 and abs(speed) < 0.82:
            for i in range(8):
                #col 13
                int = random.randint(0, 1)
                matrix[i-1, 13] = int
                int = random.randint(0, 1)

                matrix[i-1, 12] = int
                matrix[i-1, 11] = int
                for x in range(10):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.82 and abs(speed) < 0.84:
            for i in range(8):
                #col 14
                int = random.randint(0, 1)
                matrix[i-1, 14] = int
                int = random.randint(0, 1)

                matrix[i-1, 13] = int
                matrix[i-1, 12] = int
                for x in range(11):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.84 and abs(speed) < 0.86:
            for i in range(8):
                #col 15
                int = random.randint(0, 1)
                matrix[i-1, 15] = int
                int = random.randint(0, 1)

                matrix[i-1, 14] = int
                matrix[i-1, 13] = int
                for x in range(12):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.86 and abs(speed) < 0.88:
            for i in range(8):
                #col 16
                int = random.randint(0, 1)
                matrix[i-1, 16] = int
                int = random.randint(0, 1)

                matrix[i-1, 15] = int
                matrix[i-1, 14] = int
                for x in range(13):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.88 and abs(speed) < 0.9:
            for i in range(8):
                #col 17
                int = random.randint(0, 1)
                matrix[i-1, 17] = int
                int = random.randint(0, 1)

                matrix[i-1, 16] = int
                matrix[i-1, 15] = int
                for x in range(14):
                    matrix[i-1, x+1] = 1


        elif abs(speed) > 0.9 and abs(speed) < 0.92:
            for i in range(8):
                #col 18
                int = random.randint(0, 1)
                matrix[i-1, 18] = int
                int = random.randint(0, 1)

                matrix[i-1, 17] = int
                matrix[i-1, 16] = int
                for x in range(15):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.92 and abs(speed) < 0.94:
            for i in range(8):
                #col 19
                int = random.randint(0, 1)
                matrix[i-1, 19] = int
                int = random.randint(0, 1)

                matrix[i-1, 18] = int
                matrix[i-1, 17] = int
                for x in range(16):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.94 and abs(speed) < 0.96:
            for i in range(8):
                #col 20
                int = random.randint(0, 1)
                matrix[i-1, 20] = int
                int = random.randint(0, 1)

                matrix[i-1, 19] = int
                matrix[i-1, 18] = int
                for x in range(17):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.96 and abs(speed) < 0.98:
            for i in range(8):
                #col 21
                int = random.randint(0, 1)
                matrix[i-1, 21] = int
                int = random.randint(0, 1)

                matrix[i-1, 20] = int
                matrix[i-1, 19] = int
                for x in range(18):
                    matrix[i-1, x+1] = 1

        elif abs(speed) > 0.98:
            for i in range(8):
                #col 22
                for x in range(23):
                    matrix[i-1, x+1] = 1

########## CONTOURS ##########

## find the largest contour of a given color
def update_contour(color):
    global contour_center
    global contour_area
    global found_contour
    
    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # crop the image
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        # find contours of the color
        contours = rc_utils.find_contours(image, color[0], color[1])
        # get largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            #pretty self explanatory
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            
            found_contour = color
        else:
            found_contour = None
            contour_center = None
            contour_area = None

        return found_contour

########## CONE SALOM ##########

def remap_range(val: float, old_min: float, old_max: float, new_min:float, new_max:float) -> float:
    # not even gonna try
    old_range = old_max - old_min
    new_range = new_max - new_min
    return new_range * (float(val - old_min) / float(old_range)) + new_min

def update_contour_cone():
    global contour_center_cone
    global contour_area_cone
    global COLOR_CONE
    global LAST_SEEN
    image = rc.camera.get_color_image()

    if image is None:
        contour_center_cone = None
        contour_area_cone = 0
    else:
        # crop the image
        image = rc_utils.crop(image, CROP_FLOOR_CONE[0], CROP_FLOOR_CONE[1])
        # find largerst cone of a certain color
        cone_1 = rc_utils.get_largest_contour(rc_utils.find_contours(image, CONE_1[0], CONE_1[1]), MIN_CONTOUR_AREA_CONE)
        # find largest cone of another color
        cone_2 = rc_utils.get_largest_contour(rc_utils.find_contours(image, CONE_2[0], CONE_2[1]), MIN_CONTOUR_AREA_CONE)
        if cone_1 is None:
            # area of cone 1 is 0
            one_ar = 0
        else:
            # area is area of cone
            one_ar = rc_utils.get_contour_area(cone_1)
        if cone_2 is None:
            # area is none again
            two_ar = 0
        else:
            # same thing
            two_ar = rc_utils.get_contour_area(cone_2)
        if one_ar == 0 and two_ar == 0:
            # if they are both nothing, no cone
            COLOR_CONE = None
        else:
            # compares the areas and decides the largest cone
            if one_ar > two_ar:
                contour_area_cone = one_ar
                contour_center_cone = rc_utils.get_contour_center(cone_1)
                COLOR_CONE = CONE_1
            else:
                contour_area_cone = two_ar
                contour_center_cone = rc_utils.get_contour_center(cone_2)
                COLOR_CONE = CONE_2

########## LINE FOLLOWER ##########

def line_follower():
    global previous_angle, speed
    TURNBACK = 1

    kp = 0.003
    speed = 1

    TOP_Y = 295
    BOTTOM_Y = 325

    BLUE = ((90, 100, 100), (110, 255, 255))
    GREEN = ((50, 30, 80), (80, 155, 220))
    ORANGE = ((7, 29, 176), (15, 205, 220))
    COLOR_PRIORITY = (GREEN, ORANGE) # The priority is reversed

    image = rc.camera.get_color_image()
    image = rc_utils.crop(image, (TOP_Y,0), (BOTTOM_Y,640))

    scan = rc.lidar.get_samples()
    front_distance = rc_utils.get_lidar_average_distance(scan, 0, 3)
    
    if front_distance < 310:
            kp = 0.004 # 0.0029
            speed = 0.6
            
    else:
        kp = 0.0025
        speed = 1
    
    if image is not None:
        for x, color in enumerate(COLOR_PRIORITY):
            contours = rc_utils.find_contours(image, COLOR_PRIORITY[x][0], COLOR_PRIORITY[x][1])
            contour = rc_utils.get_largest_contour(contours, 10)
            contour_center = rc_utils.get_contour_center(contour)

            bottom_left_x = 0        
            bottom_left_y = 0
            bottom_right_x = 0
            bottom_right_y = 0

            if contour is not None:
                global error_position
                contour_array_size = len(contour)
                top_left_x = contour[0][0][0]
                top_left_y = contour[0][0][1]
                top_right_x = contour[contour_array_size - 1][0][0]
                top_right_y = contour[contour_array_size - 1][0][1]

                for index_l, pixel_l in enumerate(contour):
                    if contour[index_l][0][1] > bottom_left_y:
                        bottom_left_x = contour[index_l][0][0]
                        bottom_left_y = contour[index_l][0][1]
                    
                for index_r, pixel_r in enumerate(contour):
                    if contour[index_r][0][1] >= bottom_right_y:
                        bottom_right_x = contour[index_r][0][0]
                        bottom_right_y = contour[index_r][0][1]

                top_x = (top_left_x + top_right_x)/2
                bottom_x = (bottom_left_x + bottom_right_x)/2

                error_angle = (top_x - bottom_x)
                error_position = (contour_center[1] - 320)
                angle = (error_position + error_angle)*kp

                if(angle > 1):
                    angle = 1
                elif(angle < -1):
                    angle = -1
                break

            else: 
                if error_position > 0:
                    angle = TURNBACK
                else:
                    angle = -1 * TURNBACK
    
########## WALL FOLLOWER ##########

def ets(scan):
    WINDOW = 110
    RAY_WIDTH = 25
    weight_range = 150
    kp = 0.045
    speed = 0.8

    if len(scan) > 1000:
        conversion_factor = 3
    else:
        conversion_factor = 2
    
    right_scan = scan[0:WINDOW * conversion_factor]
    left_scan = scan[len(scan) - WINDOW * conversion_factor:]

    max_right_distance = 0
    max_right_index = 0
    for right_index, _ in enumerate(right_scan):
        right_distance = rc_utils.get_lidar_average_distance(scan, right_index/conversion_factor, RAY_WIDTH)
        if right_distance > max_right_distance:
            max_right_distance = right_distance
            max_right_index = right_index
    
    max_left_distance = 0
    max_left_index = len(scan)
    for left_index, _ in enumerate(left_scan):
        left_distance = rc_utils.get_lidar_average_distance(scan, (left_index/conversion_factor) + (360 - WINDOW), RAY_WIDTH)
        if left_distance > max_left_distance:
            max_left_distance = left_distance
            max_left_index = ((360 - WINDOW) * conversion_factor) + left_index
    
    right_angle = max_right_index / conversion_factor
    left_angle = ((len(scan) / conversion_factor) - (max_left_index / conversion_factor)) * -1

    if max_right_distance > max_left_distance:
        right_angle_weight = (max_right_distance - max_left_distance) / weight_range
        if right_angle_weight > 0.5:
            right_angle_weight = 0.5
        left_angle_weight = right_angle_weight * -1
    else:
        left_angle_weight = (max_left_distance - max_right_distance) / weight_range
        if left_angle_weight > 0.5:
            left_angle_weight = 0.5
        right_angle_weight = left_angle_weight * -1

    right_angle_weight += 0.5
    left_angle_weight += 0.5

    angle = (((right_angle * right_angle_weight) + (left_angle * left_angle_weight)) / 2) * kp

    if angle > 1:
        angle = 1
    elif angle < -1:
        angle = -1

    if abs(angle) > 0.1:
        speed = ((abs(angle) * -1) * 0.3) + 1
    else:
        speed = 1

    if speed > 1:
        speed = 1
    elif speed < -1:
        speed = -1
    return [speed, angle]

########## FORK ########## 

def wall_follower(left = True):
    RAY_FROM_OFFSET = 60.0
    WINDOW = 3.0
    kp = 0.0075
    target_offset = 180
    if left:
        target_offset = target_offset * -1
    speed = 0.5

    scan = rc.lidar.get_samples()

    _, right_abs_distance = rc_utils.get_lidar_closest_point(scan, (0, 180))
    _, left_abs_distance = rc_utils.get_lidar_closest_point(scan, (181, 360))
    right_offset_distance = rc_utils.get_lidar_average_distance(scan, RAY_FROM_OFFSET, WINDOW)
    left_offset_distance = rc_utils.get_lidar_average_distance(scan, (360 - RAY_FROM_OFFSET), WINDOW)

    right_math = (right_offset_distance ** 2) - (right_abs_distance ** 2)
    left_math = (left_offset_distance ** 2) - (left_abs_distance ** 2)

    if right_math < 0:
        right_math = 0
    if left_math < 0:
        left_math = 0
    
    right_parallel_distance = math.sqrt(right_math)
    left_parallel_distance = math.sqrt(left_math)

    error = (right_parallel_distance - left_parallel_distance) + target_offset
    angle = error * kp

    if angle < 0:
        angle = angle * 1.1

    if(angle > 1):
        angle = 1
    elif(angle < -1):
        angle = -1

#--------------------------------------------------------------------------------------#
# MAIN FUNCTIONS
#--------------------------------------------------------------------------------------#
def start():
    pass 

def update():

    ## Global Variables ##
    global image, speed, angle

    ###############################################################
    # MAIN CODE
    ###############################################################

    ########## WAIT FOR START ##########

    # looks for green contour and when it finds it it starts the code
    if current_level == "waiting":
        GREEN = ((30, 150, 150),(80, 255, 255))  # NEED TO TUNE
        found_contour = update_contour(GREEN)
        if found_contour is "GREEN":
            current_level = "wall_follower"
           

    ########## DETERMINE STATE ##########
    global AR_id
    global orientation

    ## AR Tag Detection ## 
    if markers is not None:
        if len(markers) > 1:
            print("more than one AR tag has been found")
        for marker in markers:
            AR_id = marker.get_id()
            orientation = marker.get_orientation()
            corners = marker.get_corners()

    ## figure out state ##
    if AR_id is not None:
        if AR_id == 0:
            #ramp
            current_level = "wall_follower"
        if AR_id == 1:
            ## decide elevator or cone salom
            current_level = "line_follower"
        if AR_id == 2:
            current_level = "cone_salom"
        if AR_id == 3:
            current_level = "fork"
            rc.physics.set_object_detection('direction_data_edgetpu.tflite', 'direction_labels.txt')

        if AR_id == 4:
            current_level = "wall_follower"

    ########## WALL FOLLOWER CODE ##########
    if current_level == "wall_follower":
        scan = rc.lidar.get_samples()
        ets_vals = ets(scan)
        speed = ets_vals[0]
        angle = ets_vals[1]

    ########## LINE FOLLOWER CODE ##########
    if current_level == "line_follower":
        line_follower()
        #line_follower code


    ########## CONE SALOM CODE ##########
    if current_level == "cone_salom":
        global findnext
        global LAST_SEEN

        # if its trying to find the next code
        if findnext:
            #updates the contour of the cone
            update_contour_cone()
            # if the cone area is less than 5000 but it finds a cone, set the setpoint to go to
            if contour_area_cone < 5000 and contour_center_cone is not None:
                setpoint = rc.camera.get_width()//2
                if COLOR_CONE == CONE_1:
                    setpoint -= 45
                else:
                    setpoint += 45
                error = setpoint - contour_center_cone[1]

                # do this thing ig to find the error
                angle = -remap_range(error, -setpoint, setpoint, -1, 1)

                angle = rc_utils.clamp(angle, -1, 1)

            # if the cone is really close and the cone is the first color
            elif COLOR_CONE == CONE_1:
                # update the cone to the last seen
                LAST_SEEN = CONE_1
                #swerve
                angle = -0.75
                # its not trying to find the next cone
                findnext = False
            elif COLOR_CONE == CONE_2:
                # if it finds cone2, same thing
                LAST_SEEN = CONE_2
                angle = 0.75
                findnext = False
        #if its not trying to find the next cone
        else:
            scan = rc.lidar.get_samples()
            isdone = False
            # figutre out if the cone is too close
            if LAST_SEEN == CONE_2 or LAST_SEEN == CONE_1:
                _, dist = rc_utils.get_lidar_closest_point(scan, (180, 290))
                if dist < 100:
                    isdone = True
            # if the cone is too close
            if isdone:
                # allows the code to find the next cone
                findnext = True
                # swerve
                if LAST_SEEN == CONE_2:
                    angle = -1
                elif LAST_SEEN == CONE_1:
                    angle = 1

        speed = 0.52
        if abs(angle) == 1:
            speed = 0.62
            
    ########## DYNAMIC OBSTACLES / SIGN DETECTION CUSTOM CODE ##########

        if current_level == "fork":
            global obj, open, past_id, counter
            change = False

            obj = rc.physics.get_object_detection()
            id, score, x0, y0, x1, y1, inference0, inference1 = obj
            print(open)
            if open:
                if id == 0:
                    wall_follower(True)
                else:
                    wall_follower(False)
            print("s: ", score)
            if score > 0.5:
                print(id)
                #change detected
                if id != past_id:
                    past_id = id
                    counter = 0
                    change = True
                counter += 1
                print("id: ", id, "counter: ", counter)
                if change:
                    print("SWITCH DETECTED")
                    open = True

            if current_level == "elevator":
                pass
                ## elevator code, need to implement on day of grand prix

            # id refers to the id in the object detector
    ########## EMERGENCY STOP ##########

    scan = rc.lidar.get_samples()
    _, crash_distance = rc_utils.get_lidar_average_distance(scan, 0, 5)

    if crash_distance < 50:
        speed = -1
        angle = -angle
        print("Backing Up")

    

    ########################################################################
    # END OF MAIN
    ########################################################################

    ########## DOT MATRIX CODE ##########

    global num, matrix, status

    # print different messages on dot matrixes 
    if rc.controller.was_pressed(rc.controller.Button.A):
        status = "Speed"
        # starts the speedometer
    
    if rc.controller.was_pressed(rc.controller.Button.B):
        status = "Text"
        matrix = np.zeros((8, 24), dtype=np.uint8)
        rc.display.show_text("See you at the finish line mcqueen!", scroll_speed = 5)
        
    if rc.controller.is_down(rc.controller.Button.X):
        status = "Text"
        matrix = np.zeros((8, 24), dtype=np.uint8)
        rc.display.show_text("kachow")
        
    if rc.controller.was_pressed(rc.controller.Button.Y):
        status = "Text"
        matrix = np.zeros((8, 24), dtype=np.uint8)
        rc.display.show_text("RBRC3")

    if status == "Speed":
        speed_display()

    rc.drive.set_speed_angle(speed, angle)

def update_slow():

    ## DOT MATRIX VARIABLES ##
    global num, matrix, status, scroll, time
    
    ########## DOT MATRIX CODE ##########
    if num == 0 and status == "Name": 
        
        time += rc.get_delta_time()
    
        if time > 0.5:
            matrix = np.zeros((8, 24), dtype=np.uint8)
            status = "Scroll"

    if status == "Name":
        name_display()
        print("displaying name")
    elif scroll == False and status == "Scroll": 
        print("scrolling")
        rc.display.show_text("Lets get ready to rumble!!!", scroll_speed = 5)
       
        scroll = True
        time = 0
    elif status != "Speed" and status != "Text": 
        time += rc.get_delta_time()
        print(time)
        if time > 1.1:
            status = "Speed"
            time = 0
            
    rc.display.set_matrix(matrix)

    ###########################################



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.set_update_slow_time(.1)
    rc.go()
