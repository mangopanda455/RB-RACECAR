"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_a.py

Title: Lab A - Printing Statements

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: Print several statements to the terminal window and gain an understanding
of how the start(), update() and update_slow() function work. Complete the lines of
code under the #TODO indicators to complete the lab.

Expected Outcome: At the end of the lab, running the script will print several
statements to the terminal window:
- Hello World!
- The sum of 523 + 910
- Welcome message including your name
- The time elapsed (in seconds) since the program starts. This message should print out
1 time every second.
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
global counter
counter = 0

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    
    # TODO Part 1: Write a print statement that will print "Hello World!" to the terminal 
    # window
    # print("______")

    # TODO Part 2: Write a print statement that will print the sum of 523 + 910 to the 
    # terminal window. Use Python to calculate the sum and automatically insert it into
    # the print statmenet using an f string.
    # print(f"The sum of 523 + 910 is: {_____}")

    # TODO Part 3: Modify the variable below to your name. Modify the print statement,
    # such that when run, the program prints "Hello {your name}, welcome to RACECAR!"
    your_name = "_____"
    # print(_____)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global counter

    # TODO Part 4: Modify the print statement below to print the amount of time that
    # has been elapsed since the program started. The print statement should read
    # "{counter} seconds have passed since the program started!"
    counter += rc.get_delta_time()
    # Uncomment this line of code when you are ready to test
    # print(_____)


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    
    # TODO Part 5: Reading the time from the update() function provides us with too
    # much information all at once. After all, the system prints information to the
    # screen 60 times a second! (approximately) Modify the print statement below to
    # print the elapsed time of the program and round the value of "counter" to 1 digit.
    # Python round() function syntax: round(value, digits)
    # print(_____)
    
    pass # Comment out this line when you are ready to test Part 5!


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
