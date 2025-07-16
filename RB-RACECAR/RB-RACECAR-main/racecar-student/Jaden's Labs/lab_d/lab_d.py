"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_d.py

Title: Lab D - Driving in Mazes

Author: jADEN tANG

Purpose: Create a script to enable semi-autonomous driving for the RACECAR. Button presses
enable a series of instructions sent to the RACECAR, which enable it to drive in various shapes.
Complete the lines of code under the #TODO indicators to complete the lab.

Expected Outcome: When the user runs the script, they are able to control the RACECAR
using the following keys:
- When the "A" button is pressed, drive in a circle
- When the "B" button is pressed, drive in a square
- When the "X" button is pressed, drive in a figure eight
- When the "Y" button is pressed, drive in any shape of your choice
"""

########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# A queue of driving steps to execute
# Each entry is a list containing (time remaining, speed, angle)
queue = []
speed = 0
angle = 0

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    # Begin at a full stop
    rc.drive.stop()

    # Begin with an empty queue
    queue.clear()

    # Print start message
    print(
        ">> Lab D - Driving in Mazes\n"
        "\n"
        "Controls:\n"
        "   A button = drive through obstacle: \"Zigzag\"\n"
        "   B button = drive through obstacle: \"Spiral\"\n"
        "   X button = drive through obstacle: \"Hallway\"\n"
        "   Y button = drive through obstacle: \"Maze\"\n"
    )


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global queue
    global speed
    global angle
    # When the A button is pressed, add instructions to drive through the obstacle "Zigzag"
    if rc.controller.was_pressed(rc.controller.Button.A):
        drive_zigzag()

    # When the B button is pressed, add instructions to drive through the obstacle "Spiral"
    if rc.controller.was_pressed(rc.controller.Button.B):
        drive_spiral()

    # When the X button is pressed, add instructions to drive through the obstacle "Hallway"
    if rc.controller.was_pressed(rc.controller.Button.X):
        drive_hallway()

    # When the Y button is pressed, add instructions to drive through the obstacle "Maze"
    if rc.controller.was_pressed(rc.controller.Button.Y):
        drive_maze()

    # TODO Part 1: Analyze the following code segment that executes instructions from the queue.
    # Fill in the blanks with the missing variable assignments and indicies according to the
    # behavior described by the comment below.

    # If the queue is not empty, follow the current drive instruction
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


# [FUNCTION] When the function is called, clear the queue, then place instructions
# inside of the queue that cause the RACECAR to drive in the zigzag
def drive_zigzag():
    global queue

    # Use this section to define and tune static variables

    queue.clear()
    queue.append([2.5, 1, 0])
    queue.append([1.25, 1, 1])
    queue.append([1, 1, 0])
    queue.append([1.25, 1, -1])
    queue.append([0.75, 1, 0])
    queue.append([0.5, -1, 0])


# [FUNCTION] When the function is called, clear the queue, then place instructions
# inside of the queue that cause the RACECAR to drive in the spiral
def drive_spiral():
    global queue

    # Use this section to define and tune static variables

    queue.clear()

    # TODO Part 3: Append the instructions into the queue that represent the RACECAR
    # driving in the "Spiral" obstacle course
    queue.append([3.5, 1, 0])
    queue.append([1.2, 1, 1])
    queue.append([3.25, 1, 0])
    queue.append([1.2, 1, 1])
    queue.append([2.75, 1, 0])
    queue.append([1.25, 1, 1])
    queue.append([1.75, 1, 0])
    queue.append([1.25, 1, 1])
    queue.append([1, 0.5, 0])


# [FUNCTION] When the function is called, clear the queue, then place instructions 
# inside of the queue that cause the RACECAR to drive through the hallway
def drive_hallway():
    global queue

    # TODO Part 4: Create constants that represent the RACECAR driving through
    # the "Hallway" obstacle course, and then append the instructions in the
    # correct order into the queue for execution
    queue.clear()
    queue.append([1, 1, 0])
    queue.append([0.75, 0.35, 1])
    queue.append([0.75, 0.75, -1])
    queue.append([1, 1, 0])
    queue.append([0.75, 0.5, -1])
    queue.append([0.75, 0.25, 1])
    queue.append([0.75, 0.25, -1])
    queue.append([0.75, 0.25, 1])
    queue.append([0.75, 1, 0])
    queue.append([0.25, -0.5, 0])
    queue.append([1, 1, 1])
    queue.append([0.25, 1, -1])
    queue.append([0.5, 1, 0])
    queue.append([1, 1, -1])



# [FUNCTION] When the function is called, clear the queue, then place instructions 
# inside of the queue that cause the RACECAR to drive in the maze
def drive_maze():
    global queue
    queue.clear()
    # TODO Part 5: Create constants that represent the RACECAR driving through
    # different parts of the maze, and then append the instructions in the
    # correct order into the queue for execution
    queue.append([5.3, 1, 0])
    queue.append([1.2, 1, -1])
    queue.append([2.5, 1, 0])
    queue.append([1.2, 1, -1])
    queue.append([2.5, -1, 0])
    queue.append([0.5, 1, 0])
    queue.append([1.2, 1, -1])


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
