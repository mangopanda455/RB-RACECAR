"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

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
import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import time

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
global contour_center, contour_area
global speed, angle
global error

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((300, 0), (rc.camera.get_height(), rc.camera.get_width()))

# HSV Color Thresholds
BLUE = ((90, 150, 150), (120, 255, 255))  # The HSV range for the color blue

global error_history # variable to store history of detected locations
hist_len = 300 # keep only 300 points ~10sec of data
error_history = [0] * hist_len # rolling queue (append to start, pop from back to remove) for error history
cmd_history = [0] * hist_len # rolling queue (append to start, pop from back to remove) for angle cmd sent out

global queue
LAGTIME = 0.25 # seconds of lag desired ### CHANGE ME ###
queue = [0] * int(LAGTIME * 60)

########################################################################################
# Functions
########################################################################################

# [FUNCTION] Function to graph data to screen - threaded
def graph_error_data():
    fig, ax = plt.subplots()
    ax2 = ax.twinx()
    error_line, = ax.plot(range(hist_len), list(error_history), label="Line Position")
    control_line, = ax2.plot(range(hist_len), list(cmd_history), color='tab:orange', label="Control Output u(t)")
    setpoint_line = ax.axhline(y=0, color='red', linestyle='--', label='Reference')

    # Set title and label
    ax.set_title("Plot of Current Error & Control Output vs. Frame #")
    ax.set_xlabel("Frame Number")
    ax.set_ylabel("Pixels (px)")
    ax2.set_ylabel("Control Output u(t)")
    ax2.set_ylim(-1, 1)

    def update_plot(frame):
        error_line.set_ydata(error_history)
        control_line.set_ydata(cmd_history)
        return error_line, control_line, setpoint_line

    # Set plot limits
    ax.set_ylim(-320, 320)
    ax.set_xlim(0, hist_len - 1)

    # Add legend
    lines   = [error_line, control_line, setpoint_line]
    labels  = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc="upper right")

    # Start animation
    ani = animation.FuncAnimation(fig, update_plot, interval=33, blit=True)
    
    # Show plot
    plt.show()

# [FUNCTION] Update the contour_center and contour_area each frame and display image
def update_contour():
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    # Crop the image to the floor directly in front of the car
    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Find all of the contours of the saved color
        contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0

        # Display the image to the screen
        rc.display.show_color_image(image)


# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    # Set initial driving speed and angle
    rc.drive.set_speed_angle(0, 0)

    e_thread = threading.Thread(target=graph_error_data)
    e_thread.start()

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed, angle
    global error

    # Process the image
    update_contour()

    # Proportional Controller
    if contour_center is not None:
        # Control Parameters
        setpoint = rc.camera.get_width() // 2
        present_value = contour_center[1] # Find the column from (row, col)
        error = setpoint - present_value

        # P-control equation
        kp = -0.0003
        kd = -0
        angle = kp * error
        if len(error_history) >= 2:
            derivative = error_history[0] - error_history[1]
        else:
            derivative = 0
        angle += kd*derivative
        
        angle = rc_utils.clamp(angle, -1, 1)

    # Drive the car
    speed = 0.25
    rc.drive.set_speed_angle(speed, queue[0])

    # Update data to error history list
    if len(error_history) >= hist_len: # if size of history greater than spec 
        error_history.pop(-1) # remove last element and proceed with append
    if contour_center is not None:
        error_history.insert(0, -error) # negate error to match sign of control
    else:
        error_history.insert(0, 0) # no detect

    # Update data to commanded history list
    if len(cmd_history) >= hist_len:
        cmd_history.pop(-1)
    cmd_history.insert(0, queue[0]) # return the current angle that is being sent out

    # Update the queue of angles being sent
    queue.pop(0) # remove first element
    queue.append(angle) # add angle to last element
    print(f"Queue: {queue}")


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