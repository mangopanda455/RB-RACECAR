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
mode = 0
mode_order = [1, 2, 1, 3, 4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1] # between walls = 1, between lines = 2, cone slalom = 3, line following = 4
BLUE = ((80, 100, 100), (125, 255, 255))
GREEN = ((35, 255, 255), (85, 255, 255))
RED = ((170, 50, 50), (10, 255, 255))
ORANGE = ((10, 100, 100), (20, 255, 255))
CROP_FLOOR = ((180, 0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_FLOOR2 = ((300, 220), (rc.camera.get_height(), rc.camera.get_width()))
MIN_CONTOUR_AREA = 1000
BLUE = ((80, 100, 100), (125, 255, 255)) 
RED = ((165, 100, 100), (10, 255, 255))
PURPLE = ((125, 255, 255), (165, 255, 255))
LAST_SEEN = 0
contour_center = None
contour_area = 0
findnext = True
angle = 0
timer = 0
seeing = False
speed = 0
queue = []

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    pass

def remap_range(val: float, old_min: float, old_max: float, new_min:float, new_max:float) -> float:
    old_range = old_max - old_min
    new_range = new_max - new_min
    if(old_range != 0):
        return new_range * (float(val - old_min) / float(old_range)) + new_min
    else:
        return 0

def getmid(input_array):
    a = 0
    b = 0
    for i in range(4):
        a += input_array[i][0]
        b += input_array[i][1]
    return b//4

def find_area(corners):
    return abs((corners[2][0] - corners[0][0]) * (corners[2][1] - corners[0][1]))

def get_markers():
    global seeing
    global mode
    global angle
    image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(image)
    if len(markers) > 0 and find_area(markers[0].get_corners()) > 3000:
        if not seeing:
            mode+=1
            setpoint = rc.camera.get_width()//2
            error = setpoint - getmid(markers[0].get_corners())
            angle = -remap_range(error, -setpoint, setpoint, -1, 1)
            seeing = True
    else:
        seeing = False

def update():
    get_markers()
    global mode
    global angle
    global speed
    global timer
    speed = 1
    if mode == 6:
        rc.drive.set_max_speed(0.2)
    else:
        rc.drive.set_max_speed(0.25)
    if mode_order[mode] == 1:
        scan = rc.lidar.get_samples()
        _, right_distance = rc_utils.get_lidar_closest_point(scan, (40, 50))
        _, left_distance = rc_utils.get_lidar_closest_point(scan, (310, 320))
        if right_distance * 10 < left_distance:
            angle = 3 * remap_range((right_distance-30), 0, right_distance, -1, 1)
        elif left_distance * 10 < right_distance:
            angle = 3 * remap_range(-(left_distance-30), -left_distance, 0, -1, 1)
        else:
            angle = 3 * remap_range(right_distance-left_distance, -left_distance, right_distance, -1, 1)
        if angle < -1:
            angle = -1
        if angle > 1:
            angle = 1
        rc.drive.set_speed_angle(speed, angle)
    elif mode_order[mode] == 2:
        update_contour_3()
        setpoint = (rc.camera.get_width()-220)//2
        if contour_center is not None:
            error = setpoint - (contour_center[1] - 50)
            angle = 3*-remap_range(error, -setpoint, setpoint, -1, 1)
            if angle < -1:
                angle = -1
            if angle > 1:
                angle = 1
        rc.drive.set_speed_angle(1, angle)
    elif mode_order[mode] == 3:
        global findnext
        global LAST_SEEN
        if findnext:
            update_contour()
            if contour_area < 6000 and contour_center is not None:
                setpoint = rc.camera.get_width()//2
                error = setpoint - contour_center[1]
                angle = -remap_range(error, -setpoint, setpoint, -1, 1)
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
            angles, distance = rc_utils.get_lidar_closest_point(scan)
            isdone = False
            if LAST_SEEN == RED:
                if angles <= 300 and angles >= 180:
                    isdone = True
            elif LAST_SEEN == BLUE:
                if angles >= 60 and angles <= 180:
                    isdone = True
            if isdone and distance < 10000:
                findnext = True
                if LAST_SEEN == RED:
                    angle = -1
                else:
                    angle = 1
        rc.drive.set_speed_angle(speed, angle)
    elif mode_order[mode] == 4:
        update_contour_2()
        if contour_center is not None:
            setpoint = rc.camera.get_width()//2
            error = setpoint - contour_center[1]
            angle = 2*-remap_range(error, -setpoint, setpoint, -1, 1)
            if(angle < -1):
                angle = -1
            if(angle > 1):
                angle = 1
            #line follower
            rc.drive.set_speed_angle(speed, angle)
        else:
            mode += 1
            pass
            

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
        

OverallColor = None
COLORS = [RED, BLUE, GREEN]
def update_contour_2():
    global contour_center
    global contour_area
    global OverallColor
    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found
        if OverallColor is None:
            for color in COLORS:
                contours = rc_utils.find_contours(image, color[0], color[1])
                largestc = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
                OverallColor = color
                if largestc is not None:
                    contour_center = rc_utils.get_contour_center(largestc)
                    contour_area = rc_utils.get_contour_area(largestc)
                    rc_utils.draw_contour(image, largestc)
                    rc_utils.draw_circle(image, contour_center)
                    break
        else:
            contours = rc_utils.find_contours(image, OverallColor[0], OverallColor[1])
            largestc = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
            if largestc is not None:
                contour_center = rc_utils.get_contour_center(largestc)
                contour_area = rc_utils.get_contour_area(largestc)
                rc_utils.draw_contour(image, largestc)
                rc_utils.draw_circle(image, contour_center)
            else:
                contour_center = None
                contour_area = 0
        # Display the image to the screen
        rc.display.show_color_image(image)

COLOR2 = [ORANGE, PURPLE]
last_center = None
def update_contour_3():
    rc.drive.set_max_speed(0.2)
    global contour_center
    global contour_area
    global last_center
    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        image = rc_utils.crop(image, CROP_FLOOR2[0], CROP_FLOOR2[1])
        center = 0
        largest_area = 0
        for color in COLOR2:
            contours = rc_utils.find_contours(image, color[0], color[1])
            largestc = rc_utils.get_largest_contour(contours)
            if largestc is not None:
                if rc_utils.get_contour_area(largestc) > largest_area:
                    center = rc_utils.get_contour_center(largestc)
                    largest_area = rc_utils.get_contour_area(largestc)
        if last_center is not None:
            if abs(contour_center[1] - last_center[1] < 10):
                contour_center = center
                last_center = contour_center
        else:
            last_center = contour_center
            contour_center = center
        rc.display.show_color_image(image)


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
