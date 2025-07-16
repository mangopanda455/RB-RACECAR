"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_b.py

Title: Lab B - Printing Statements Using Controller

Author: Jaden Tang

Purpose: Print several statements to the terminal window from the update() function
and gain an understanding of how to poll data from the controller. Complete the lines of
code under the #TODO indicators to complete the lab.

Expected Outcome: At the end of the lab, running the script will print several
statements to the terminal window based on controller input:
- When the "A" button is pressed, print "Hello World!" to the terminal window once
- When the "B" button is released, print "Welcome to RACECAR <your name>!" once
- When the "X" button is pressed, print the current elapsed time of the script to the 
terminal window in seconds. The script should continue printing time updates if the
button is held down. Round the time to 2 decimal places.
- When the "Y" button is pressed, print the current elapsed time of the script to the
terminal window in seconds. The script should only print out the time once after the 
button has been pressed, regardless if it is held down or not. Round the time to 2
decimal places.
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

# Declare any global variables here
global your_name
global counter
counter = 0

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global your_name

    # TODO Part 1: Modify the variable below to your name. Modify the print statement,
    # such that when run, the program prints "Hello {your name}, welcome to RACECAR!"
    your_name = "Jaden Tang"
    print(f"Welcome {your_name}, welcome to RACECAR!")

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global counter
    counter += rc.get_delta_time()

    # TODO Part 2: Modify the following if statement's contents to print out "Hello World!"
    # to the terminal window after the "A" button has been pressed on the keyboard.
    if rc.controller.was_pressed(rc.controller.Button.A):
        print("Hello World!")

    # TODO Part 3: Modify the following if statement to print out "Welcome to RACECAR
    # <your name>!" after the "B" button has been released on the keyboard.
    if rc.controller.was_released(rc.controller.Button.B):
        print(f"Welcome to RACECAR {your_name}!")

    # TODO Part 4: Modify the following if statment to print out the current elapsed 
    # time of the script to the terminal window in seconds when the X button is pressed. 
    # The script should continue printing time updates if the button is held down. 
    # Round the time to 2 decimal places.
    if rc.controller.is_down(rc.controller.Button.X):
        print(f"The current script has been running for {round(counter,2)} seconds!")

    # TODO Part 5: Create an if statement below to print out the current elapsed
    # time of the script to the terminal window in seconds when the Y button is pressed.
    # The script should only print out the time once after the button has been pressed, 
    # regardless if it is held down or not. Round the time to 2 decimal places.
    if rc.controller.was_pressed(rc.controller.Button.Y):
        print(f"The script has been running for {round(counter, 2)} seconds!")

    

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
