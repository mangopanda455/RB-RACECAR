"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: demo.py

Title: Demo RACECAR program

Purpose: To verify that basic RACECAR functions work properly and the student has set up
the system correctly to run Python scripts with the RACECAR start/update paradigm in the
Unity simulator.

Expected Outcome: Terminal output and RACECAR movement occurs when buttons are pressed
- When the "A" button is pressed, print a message to the terminal window
- When the "B" button is pressed, RACECAR moves forward and to the right
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, '../../library')
import racecar_core

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
counter = 0
isDriving = False

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    # If we use a global variable in our function, we must list it at
    # the beginning of our function like this
    global counter
    global isDriving

    # The start function is a great place to give initial values to global variables
    counter = 0
    isDriving = False

    # This tells the car to begin at a standstill
    rc.drive.stop()

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    
    global counter
    global isDriving

    # This prints a message every time the A button is pressed on the controller
    if rc.controller.was_pressed(rc.controller.Button.A):
        print("The A button was pressed")

    # Reset the counter and start driving in an L every time the B button is pressed on
    # the controller
    if rc.controller.was_pressed(rc.controller.Button.B):
        counter = 0
        isDriving = True # Toggle Variable

    if isDriving:
        # rc.get_delta_time() gives the time in seconds since the last time
        # the update function was called
        counter += rc.get_delta_time()

        if counter < 1:
            # Drive forward at full speed for one second
            rc.drive.set_speed_angle(1, 0)
        elif counter < 2:
            # Turn right at full speed for the next second
            rc.drive.set_speed_angle(1, 1)
        elif counter < 3:
            #Turn left at full speed for the next second
            rc.drive.set_speed_angle(1, -1)
        else:
            # Otherwise, stop the car
            rc.drive.stop()
            isDriving = False

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # This prints a message every time that the right bumper is pressed during
    # a call to to update_slow.  If we press and hold the right bumper, it
    # will print a message once per second
    if rc.controller.is_down(rc.controller.Button.RB):
        print("The right bumper is currently down (update_slow)")


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
