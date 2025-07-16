"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: arCODEreader.py 

Title: Ar Code Reader

Author: Jaden Tang

Purpose: read AR codes

Expected Outcome: read AR Codes
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2


# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(0, '../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
CROP_FLOOR = ((180, 0), (rc.camera.get_height(), rc.camera.get_width()))

speed = 0.0
angle = 0.0
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
BLUE = ((90, 115, 115),(120, 255, 255), "BLUE")
GREEN = ((40, 115, 115),(80, 255, 255), "GREEN")
RED = ((170, 115, 115),(10, 255, 255), "RED")
COLORS = [BLUE, GREEN, RED] 
MIN_CONTOUR_AREA = 30
########################################################################################
# Functions
########################################################################################

class ARMarker:
    def __init__(self, marker_id, marker_corners, orientation, area):
        self.id = marker_id
        self.corners = marker_corners
        self.orientation = orientation
        self.area = area 


# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    pass # Remove 'pass' and write your source code for the start() function here

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
        for colors in COLORS:
            contours = rc_utils.find_contours(image, colors[0], colors[1])
            if contours is not None:
                for contour in contours:
                    if contour is not None:
                        print(contour)
                        contour_area = rc_utils.get_contour_area(contour)
                        rc_utils.draw_contour(image, contour)
        # Display the image to the screen
        rc.display.show_color_image(image)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    image = rc.camera.get_color_image()
    markers, image = detect_AR_Tag(image)
    # Print the corners and id of the first detected marker to the terminal
    print(f"========== Detection Summary ==========")
    print(f"Amount of AR Tags Found: {len(markers)}")
    for marker in markers:
        print(f"Marker ID: {marker.id} || Marker Orientation: {marker.orientation} || Marker Area: {marker.area}")

    print(f"========== End of Summary ==========\n")
    update_contour()
    if rc.controller.get_trigger(rc.controller.Trigger.LEFT) > 0:
        speed = -1
    elif rc.controller.get_trigger(rc.controller.Trigger.RIGHT) > 0:
        speed = 1
    else:
        speed = 0
    (x, y) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    if x > 0.5:
        angle = 1
    elif x < -0.5:
        angle = -1
    else:
        angle = 0
    rc.drive.set_speed_angle(speed, angle)

def detect_AR_Tag(image):
    markers = []
    corners, ids, _ = detector.detectMarkers(image)
    for x in range(len(corners)):
        current_corners = corners[x][0]
        orientation = ""
        if current_corners[0][0] == current_corners[1][0]: # if x1 = x2, RIGHT or LEFT
            if current_corners[0][1] > current_corners[1][1]: # if y1 > y2, LEFT
                orientation = "LEFT"
            else: # if y2 > y1, RIGHT
                orientation = "RIGHT"
        else: 
            if current_corners[0][0] > current_corners[1][0]: # if x1 > x2, DOWN
                orientation = "DOWN"
            else: # if x2 > x1, UP
                orientation = "UP"
        area = (current_corners[2][0] - current_corners[0][0]) * (current_corners[2][1] - current_corners[0][1])

        current_marker = ARMarker(ids[x][0], current_corners, orientation, abs(area))
        markers.append(current_marker)

    cv2.aruco.drawDetectedMarkers(image, corners, ids, (0, 255, 0))

    return markers, image


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
