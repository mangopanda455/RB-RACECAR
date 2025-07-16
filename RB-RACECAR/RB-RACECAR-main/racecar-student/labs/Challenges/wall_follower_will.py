
"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_i.py

Title: Lab I - Wall Follower

Author: Will Alvini

Purpose: This script provides the RACECAR with the ability to autonomously follow a wall.
The script should handle wall following for the right wall, the left wall, both walls, and
be flexible enough to handle very narrow and very wide walls as well.

Expected Outcome: When the user runs the script, the RACECAR should be fully autonomous
and drive without the assistance of the user. The RACECAR drives according to the following
rules:
- The RACECAR detects a wall using the LIDAR sensor a certain distance and angle away.
- Ideally, the RACECAR should be a set distance away from a wall, or if two walls are detected,
should be in the center of the walls.
- The RACECAR may have different states depending on if it sees only a right wall, only a 
left wall, or both walls.
- Both speed and angle parameters are variable and recalculated every frame. The speed and angle
values are sent once at the end of the update() function.

Note: This file consists of bare-bones skeleton code, which is the bare minimum to run a 
Python file in the RACECAR sim. Less help will be provided from here on out, since you've made
it this far. Good luck, and remember to contact an instructor if you have any questions!

Environment: Test your code using the level "Neo Labs > Lab I: Wall Follower".
Use the "TAB" key to advance from checkpoint to checkpoint to practice each section before
running through the race in "race mode" to do the full course. Lowest time wins!
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
speed = 0
angle = 0


########################################################################################
# Functions
########################################################################################

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


# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed 
    global angle

    speed = 0.5
    angle = 0
    print("start")


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global angle
    global speed
    global left_distance
    global right_distance
    

   
    straight_distance = distance_straight()
    left_distance = distance_left()
    right_distance = distance_right()
   
    


    if straight_distance > 80:
        speed = 1
    elif straight_distance > 60:
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
    
           
           

    print(speed)
    print(f"front: {straight_distance}")
    print(abs(left_distance - right_distance))
    

    image = rc.camera.get_color_image()
    rc.display.show_color_image(image)
   
    rc.drive.set_speed_angle(speed, angle)

    # print("-" * 50)

    # print(f"front distance: {front_distance}")
    # print(f"left distance: {left_distance}")
    # print(f"right distance: {right_distance}")
    # print(left)
    # print(right)


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    global left_distance
    global right_distance
    global front_distance
    # print("-" * 50)

    # print(f"front distance: {front_distance}")
    # print(f"left distance: {left_distance}")
    # print(f"right distance: {right_distance}")

    pass  # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()


