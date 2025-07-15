
"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: grand_prix.py

Title: Grand Prix Day!

Author: Will Alvini

Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR will traverse the obstacle course autonomously without human intervention.
Once the start button is pressed, the RACECAR must drive through the course until it
reaches finish line.

Note: There is no template code in this document to follow except for the RACECAR script 
structure found in template.py. You are expected to use code written from previous labs
to complete this challenge. Good luck!

Expected Outcome: When the user runs the script, they must not be able to manually control
the RACECAR. The RACECAR must move forward on its own, traverse through the course, and then
stop on its own.
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 
import numpy as np 


# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

STAGES = {1: "LIDAR", 2: "PATH", 3: "LINE", 4: "CONES"}
current_stage = STAGES[1]

speed = 0
angle = 0


rc = racecar_core.create_racecar()


########### AR TAGS #########
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
########### CONTOURS #################

CROP_FLOOR = ((200, 0), (rc.camera.get_height(), rc.camera.get_width()))
MIN_CONTOUR_AREA = 40
MIN_CONTOUR_AREA_CONES = 500

contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
found_contour = None

contours = None

########## CONES #################

found_cone = None
STATES = {1: "APPROACH", 2: "TURN", 3: "RESET", 4: "SEARCH"}
current_state = STATES[1]

#-------- HSV Ranges ---------#
RED = ((165, 50, 50),(179, 255, 255)) 
ORANGE = ((10, 150, 150),(20, 255, 255)) 
YELLOW = ((21, 50, 50),(30, 255, 255)) 
GREEN = ((30, 150, 150),(80, 255, 255))  
BLUE = ((80, 150, 150), (125, 255, 255))  
PURPLE = ((125, 50, 50),(165, 255, 255)) 

WHITE = ((0,0,200),(179, 100,255))
#------------------------------#

############

# Declare any global variables here






########################################################################################
# Functions
########################################################################################
########## AR TAG ####################
def detect_AR_Tag(image):
    corners, ids, _ = detector.detectMarkers(image)
    return corners, ids
######## LIDAR ############
def distance_left():
    scan = rc.lidar.get_samples()

    _ , distance = rc_utils.get_lidar_closest_point(scan, (315, 360)) 
    distance = distance - 19

    return distance 

def distance_right():
    scan = rc.lidar.get_samples()

    _ , distance = rc_utils.get_lidar_closest_point(scan, (0, 45)) 
    distance = distance - 19

    return distance 

def distance_straight():
    scan = rc.lidar.get_samples()

    _ , distance = rc_utils.get_lidar_closest_point(scan, (355, 5)) 
    distance = distance - 19

    return distance 

def left_hard():
    scan = rc.lidar.get_samples()

    _ , distance = rc_utils.get_lidar_closest_point(scan, (300, 315)) 
    distance = distance - 19

    return distance 

def right_hard():
    scan = rc.lidar.get_samples()

    _ , distance = rc_utils.get_lidar_closest_point(scan, (45, 60)) 
    distance = distance - 19

    return distance 
################## CONTOURS ##########################

def update_contour_line(color): ## finds a contour of a specific color, returns info and draws it
    global contour_center
    global contour_area
    global found_contour
    global image
    global contour_list 

    contour_list = []
    centers = []

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

       

        #retrieve and crop image
    

        #extract colors from image#
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #make a mask based on the hsv lower and upper values
        mask = cv2.inRange(hsv, color[0], color[1])

        # find contours

        #only finds contours based on color described in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > MIN_CONTOUR_AREA:
                contour_list.append(contour)
                center = rc_utils.get_contour_center(contour)
                centers.append(center)
                
        
        if len(centers) > 1:
        
            contour_center = (int((abs(centers[0][0] + centers[1][0])) / 2), int((abs(centers[0][1] + centers[1][1])) / 2))
            found_contour = color
            
        if len(contour_list) == 0:
            found_contour = None
            
            
def determine_orientation(corners):
    
    

    #print(f"TagN when orientation: {TagN}")

    firstx = corners[0][0][0][0]
    firsty = corners[0][0][0][1]
    secondx = corners[0][0][1][0]
    secondy = corners[0][0][1][1]

    orientation = None


    if firstx < secondx and firsty == secondy:
        orientation = "up"

    elif firstx == secondx and firsty < secondy:
        orientation = "right"

    elif firstx > secondx and firsty == secondy:
        orientation = "down"

    elif firstx == secondx and firsty > secondy:
        orientation = "left"

    return orientation

    
def update_contour(color):
    global contour_center
    global contour_area
    global found_contour
    global image
    global contours

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
        

        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        

        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
            found_contour = color
        else:
            found_contour = None
            contour_center = None
            contour_area = None

        rc.display.show_color_image(image)
        
   

def update_contour_color(hsv_lower, hsv_upper):
    global contour_center
    global contour_area

    #retrieve and crop image
    image = rc.camera.get_color_image()
    image = rc_utils.crop(image, (100, 0), (rc.camera.get_height(), rc.camera.get_width()))

    #extract colors from image#
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #make a mask based on the hsv lower and upper values
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)

    # find contours

    #only finds contours based on color described in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    contour_max = []

    if len(contours) > 0:
        contour_max = contours[0]
    CONTOUR_MIN = 949

        

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found

        #figure out biggest contour
        
        for contour in contours:
            if cv2.contourArea(contour) > CONTOUR_MIN:
                if cv2.contourArea(contour) > cv2.contourArea(contour_max): 
                    contour_max = contour
                    

    
    if len(contours) > 0 and len(contour_max) > 0:
        if contour_max is None:
            print("Contour max is None")
            return([])
        elif len(contour_max) == 0:
            print("Contour max is an empty list")
        else:
           
            return(contour_max)


##########################################################################
# CONES
#######################################################################

def detect_largest_color():
    
    contour_larg = None #initialize to None
    global image
    contour_color = None

    

    image = rc.camera.get_color_image()
    image = rc_utils.crop(image, (100, 0), (rc.camera.get_height(), rc.camera.get_width()))


    red_c = update_contour_color(RED[0], RED[1])
    
    blue_c = update_contour_color(BLUE[0], BLUE[1])
    

    color_l = [blue_c, red_c]

    for color in color_l:
        if contour_larg is None:
            
            contour_larg = color
            if color is red_c:
                contour_color = "RED"
            if color is blue_c:
                contour_color = "BLUE"
            #print("color")
        else:
            pass
    for color in color_l:
        if contour_larg is not None:
            if isinstance(color, np.ndarray):
                
                if cv2.contourArea(color) >= cv2.contourArea(contour_larg):
                    contour_larg = color
                    if color is red_c:
                        contour_color = "RED"
                    if color is blue_c:
                        contour_color = "BLUE"


    if contour_larg is not None:
        cv2.drawContours(image, [contour_larg], -1, (255, 255, 255), 3)
        
    print(contour_color)
    return contour_color

def update_contour_cones(color):
    global contour_center
    global contour_area
    global found_contour
    global image

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
        

        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA_CONES)

        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
            found_contour = color
        else:
            found_contour = None
            contour_center = None
            contour_area = None
        
    rc.display.show_color_image(image)

def check_pass():
    scan = rc.lidar.get_samples()

    _ , distance = rc_utils.get_lidar_closest_point(scan, (90, 270)) 
    distance = distance - 19

    return distance


def find_crash_test():
    scan = rc.lidar.get_samples()

    _ , distance = rc_utils.get_lidar_closest_point(scan, (280, 80)) 
    distance = distance - 19

    return distance

def find_distance(): # find distance in front
    
    scan = rc.lidar.get_samples()

    _ , distance = rc_utils.get_lidar_closest_point(scan, (340, 20)) 
    distance = distance - 19

    return distance

def update_contour_floor(color):
    global contour_center
    global contour_area
    global found_contour
    global image
    global contours

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, (300, 0), (rc.camera.get_height(), rc.camera.get_width()))

        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found

        contours = rc_utils.find_contours(image, color[0], color[1])

        # Display the image to the screen
        

        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        

        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
            found_contour = color
        else:
            found_contour = None
            contour_center = None
            contour_area = None

        rc.display.show_color_image(image)

# AR TAGS ###
def find_area(corners):
    
    
    return abs((corners[0][0][2][0] - corners[0][0][0][0]) * (corners[0][0][2][1] - corners[0][0][0][1]) )


# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global image 
    global on_path
    global path_contour
    global found_cone
    global current_state
    global past
    global line_contour
    global linechange

    linechange = False
   
    global last_color

    line_contour = None

    image = None

    past = False

    on_path = False
    path_contour = None

    current_state = STATES[1]
    found_cone = None
    print("New Level")
    last_color = "NONE"

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():

    

    ######## VARIABLES ################
    
    global angle
    global speed
    global image
    global current_stage
    global line_contour

    global linechange

    ## LIDAR ##
    global left_distance
    global right_distance
    ## CV ##
    global contour_area
    global contour_center
    global found_contour
    global contour_list

    area = None
    
    global past 

    global on_path

    image = rc.camera.get_color_image()
    
    #########################################



    ####### AR Tags #####
    corners, ids = detect_AR_Tag(image)

    if len(corners) > 0:
        orientation = determine_orientation(corners)
        area = find_area(corners)
        
    ## 32 down (bottom right) s line in between (purple/oranmge), 32 up is lidar ntural, 32 right is cone follower , 32 up is line follower (lime/red) (or maybe just whatevers first), 32 up again is lidar, then lidar again (natural but after man made), 
    
    ######################


    ########################################
    # STAGES      
    ########################################

    if current_stage == "LIDAR":
    
        straight_distance = distance_straight()
        left_distance = distance_left()
        right_distance = distance_right()
        print(left_distance)
        print(right_distance)

        left = left_hard()
        right = right_hard()



        if straight_distance > 80:
            speed = 1
        elif straight_distance > 70:
            speed = 0.5
        elif straight_distance > 50:
            speed = 0.2

        if left_distance > right_distance:
            angle = -0.7
        if right_distance > left_distance:
            angle = 0.7

        if abs(left_distance - right_distance) < 2:
            angle = 0
    
        if straight_distance < 45:
            if left_distance > right_distance:
                angle = -1
            if right_distance > left_distance:
                angle = 1

            if abs(left_distance - right_distance) < 1:
                angle = 0

       
        
        if left > 500 or right > 500:
            if left > 500: 
                
                angle = 0.8
                
                if right_distance < 10:
                    print(right_distance)
                    angle = -0.8
                    
            if right > 500: 
                
                angle = -0.8
                
                if left_distance < 10:
                    print(left_distance)
                    angle = 0.8
                    
        
          
        

        if ids is not None and len(ids) > 0 and ids[0] == 32 and orientation == "down":
            
            update_contour_line(PURPLE)
            
            if found_contour is not None:
                print("State Changed")
                current_stage = "PATH"

            update_contour_line(ORANGE)
            
            if found_contour is not None:
                print("State Changed")
                current_stage = "PATH"

            update_contour(BLUE)
            print(contour_area)
            if found_contour is not None and contour_area > 450:
                
                print("State Changed")
                current_stage = "CONES"
        if ids is not None and len(ids) > 0 and ids[0] == 32 and orientation == "up":
            if area > 3000:
                angle = 1

    
    if current_stage == "PATH":
        global path_contour
        speed = 1
        

        if path_contour is None: ## figure out which color to follow
            update_contour_line(PURPLE)
            path_contour = found_contour

            update_contour_line(ORANGE)
            if found_contour is not None:
                path_contour = found_contour
            

        if path_contour is not None:
            update_contour_line(path_contour)
            on_path = True

            if contour_center is not None:
                setpoint = rc.camera.get_width() // 2 # cemter of tje screen

                current_value = contour_center[1]

                kp = -0.003125

                error = setpoint - current_value

                angle = kp * error

                angle = rc_utils.clamp(angle, -1, 1)

        
        if on_path == True and found_contour is None:
            angle = 1
    
        if ids is not None and len(ids) > 0 and ids[0] == 32 and orientation == "up":
            print(path_contour)
            update_contour(path_contour)

            if found_contour is None:
                print("State Changed")
                current_stage = "LIDAR"
    
        

    if current_stage == "LINE":
        
        speed = 0.7

        if ids is None:
            
            color_list = [GREEN, BLUE, ((0, 50, 50),(10, 255, 255))]
            past = True

            if line_contour == None: ## find the color
                
                for color in color_list:


                    update_contour(color)
                    

                    if found_contour is not None:
                        line_contour = found_contour

                

            if line_contour is not None:
                update_contour(line_contour)

                if contour_center is not None:
                    setpoint = rc.camera.get_width() // 2 # cemter of tje screen

                    current_value = contour_center[1]

                    kp = -0.003125

                    error = setpoint - current_value

                    angle = kp * error

                    angle = rc_utils.clamp(angle, -1, 1)

        if past == True:

            if line_contour is not None:
                update_contour(line_contour)
                if found_contour is None:
                    print("State Changed")
                    current_stage = "LIDAR"
        

    if current_stage == "CONES":
        global found_cone
        global distance
        global current_state
        global last_color

        distance = find_distance()

        if found_cone is None:
            found_cone = detect_largest_color() # detects largest blue or red
    
        if current_state == "APPROACH": 
            ## whichever cone color is found, highlight it
            if found_cone == "BLUE":
                update_contour_cones(BLUE)
            if found_cone == "RED":
                update_contour_cones(RED)
    
            if contour_center is not None: ## get the contour to the center of the screen
                if found_cone == "BLUE":
                    setpoint = (rc.camera.get_width() // 2) + 100 # cemter of tje screen, slightly to the right so it passes more on left

                    current_value = contour_center[1]

                    kp = -0.003125

                    error = setpoint - current_value

                    angle = kp * error

                    angle = rc_utils.clamp(angle, -1, 1)
                if found_cone == "RED":
                    setpoint = (rc.camera.get_width() // 2) - 100 # cemter of tje screen, opposite of other

                    current_value = contour_center[1]

                    kp = -0.003125

                    error = setpoint - current_value

                    angle = kp * error

                    angle = rc_utils.clamp(angle, -1, 1)
            else:
                current_state = "SEARCH"

            speed = 0.7
    
            if distance < 55: ## when it gets close enough, turn
                print("Changing States To Turning")
                current_state = "TURN"

                
        if current_state == "TURN":

            if found_cone == "BLUE":
                update_contour_cones(BLUE)
                speed = 0.6
                angle = -1
                last_color = "BLUE"
            
            elif found_cone == "RED":
                update_contour_cones(RED)
                speed = 0.6
                angle = 1
                last_color = "RED"

            ## perform a check to see if the cone has been passed
            passcheck = check_pass()
            if passcheck < 50:
                angle = -angle
                current_state = "APPROACH"
                print("cone has been passed")


        if current_state == "RESET":
        
            print("Detecting New Cone")
            current_state = "APPROACH"

        if current_state == "SEARCH":
            if last_color == "RED":

                if found_cone == "RED":
                    found_cone = None # only search for blue contours
                    print("reset red contour")
                    angle = -1

                elif found_cone == "BLUE":
                    current_state = "APPROACH"
            elif last_color == "BLUE":
                if found_cone == "BLUE":
                    print("reset blue contour")
                    found_cone = None # only search for red contours
                    angle = 1

                elif found_cone == "RED":
                    current_state = "APPROACH"
        #figure out what current cone shuld be lookign for

     ## failsafe for if too close
        crash_test = find_crash_test()
        if crash_test < 10 and speed == 0.6: ## turn speed
            print("driving backward slow")
            speed = -0.5
            angle = 0
        elif crash_test < 30 and speed == 0.7: ## approach speed
            speed = -1
            angle = 0
            print("driving backward quickly")

       

        if ids is not None and len(ids) > 0 and ids[0] == 32 and area > 900:
            # print(area)

            update_contour_floor(WHITE)
            print(contour_area)

            if contour_area is not None and contour_area > 19000:
                        
                current_stage = "LINE"
            
    
    #### draw image ####
    cv2.aruco.drawDetectedMarkers(image, corners, ids, (0, 255, 0)) ## draw ar tags
    
    #######
    if found_contour is not None:
        
        rc_utils.draw_circle(image, contour_center)
        
        
    ######
   
    rc.display.show_color_image(image)
    rc.drive.set_speed_angle(speed, angle)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    print(f"Current Stage: {current_stage}")

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()


