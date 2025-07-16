"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-oneshot-labs

File Name: hsv-p_tuner.py

Title: HSV-P Tuner

Author: Chris Lai (MITLL)

Purpose: User is able to modify the HSV thresholds to create their own filters
and affect the precision of line following in the RACECAR. Codebase uses custom
tk UI with six trackbars to modify the lower and upper bounds of HSV values.

Allow for proportional controller tuning of a visual-servo based line follower.
Graphically display system parameters for a basic control system
"""

import sys
import cv2 as cv
import numpy as np
import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

# Create RACECAR object
rc = racecar_core.create_racecar()

# Default Variables
global H_low, H_high, S_low, S_high, V_low, V_high
H_low, H_high = 0, 179
S_low, S_high = 0, 255
V_low, V_high = 0, 255

COLOR_THRESH = ((H_low, S_low, V_low), (H_high, S_high, V_high)) # changes on user save
CROP_FLOOR = ((180, 0), (rc.camera.get_height(), rc.camera.get_width()))

global tune_speed, tune_angle
tune_speed = 0
tune_angle = 0

CONTROL_PARAM = (tune_speed, tune_angle) # changes on user save

global loc_history # variable to store history of detected locations
hist_len = 300 # keep only 300 points ~10sec of data
loc_history = [0] * hist_len # rolling queue (append to start, pop to remove)

speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

MIN_CONTOUR_AREA = 30


# Function to adjust values (you can replace these functions with actual processing logic)
def on_low_h_change(val):
    global H_low
    H_low = int(float(val))
    print(f"H_low: {H_low}")


def on_low_s_change(val):
    global S_low
    S_low = int(float(val))
    print(f"S_low: {S_low}")


def on_low_v_change(val):
    global V_low
    V_low = int(float(val))
    print(f"V_low: {V_low}")


def on_high_h_change(val):
    global H_high
    H_high = int(float(val))
    print(f"H_high: {H_high}")


def on_high_s_change(val):
    global S_high
    S_high = int(float(val))
    print(f"S_high: {S_high}")


def on_high_v_change(val):
    global V_high
    V_high = int(float(val))
    print(f"V_high: {V_high}")

def on_speed_change(val): # val is between 0 and 100% -> map to 0 to 1
    global tune_speed
    tune_speed = float(val)/100 

def on_angsens_change(val): # val is between 0 and 100% -> map to 0 to 1
    global tune_angle
    tune_angle = float(val)/100

# [FUNCTION] Create a GUI (tkinter) to dynamically adjust HSV values
def create_gui():
    root = tk.Tk()
    root.title("HSV Filter Tuner")
    root.geometry("400x700")
    root.configure(background='black')

    # Create custom fonts
    title_font = tkfont.Font(family="Helvetica", size=16, weight="bold")
    subtitle_font = tkfont.Font(family="Helvetica", size=12, weight="bold", slant="italic")

    # Create and pack the title label
    title_label = tk.Label(root, text="RACECAR Neo Demo UI", background='black', foreground='red', font=title_font)
    title_label.pack()

    # Create and pack the subtitle label
    subtitle_label = tk.Label(
        root,
        text="Line Following Tuner App",
        background='black',
        foreground='orange',
        font=subtitle_font
    )
    subtitle_label.pack()

    # Configure the ttk style for a dark theme
    style = ttk.Style()
    style.theme_use('clam')  # 'clam' is generally easier to customize

    # Define a custom font
    custom_font = tkfont.Font(family="Helvetica", size=10, weight="bold")

    # Configure the background and foreground of the Scale widget and labels
    style.configure(
        "Horizontal.TScale",
        background='black',
        troughcolor='grey',
        bordercolor='black',
        lightcolor='black',
        darkcolor='black',
        arrowcolor='white'
    )
    style.configure("TLabel", background='black', foreground='white', font=custom_font)
    style.configure("Outline.TFrame", background='black', borderwidth=2, relief='solid')

    # Attempt to change the thumb using element create
    style.element_create('custom.Horizontal.Scale.slider', 'from', 'default')
    style.layout('Custom.Horizontal.TScale', [
        ('Horizontal.Scale.trough', {
            'sticky': 'nswe',
            'children': [('custom.Horizontal.Scale.slider', {'side': 'left', 'sticky': ''})]
        })
    ])
    style.configure(
        'Custom.Horizontal.TScale',
        sliderrelief='flat',
        sliderlength=30,
        slidershadow='black',
        slidercolor='blue'
    )

    # Adjust styles to include black borders and red trough
    style.configure("Horizontal.TScale", background='black', troughcolor='grey', bordercolor='white')

    # Function to update label
    def update_label(label, value):
        label.config(text=f"{int(float(value))}")

    # Creating a scale widget with a frame
    def create_scale(root, label, from_, to, start, command):
        frame = ttk.Frame(root, style="Outline.TFrame")
        frame.pack(fill='x', expand=True, pady=10)

        min_label = tk.Label(frame, text=str(from_), bg='black', fg='white', font=custom_font)
        min_label.pack(side='left')

        max_label = tk.Label(frame, text=str(to), bg='black', fg='white', font=custom_font)
        max_label.pack(side='right')

        current_value_label = tk.Label(frame, text=str(from_), bg='black', fg='white', font=custom_font)
        current_value_label.pack(side='top')

        scale = ttk.Scale(
            frame,
            from_=from_,
            to=to,
            orient='horizontal',
            style="Horizontal.TScale",
            command=lambda v, l=current_value_label: (command(v), update_label(l, v))
        )
        scale.set(start)  # Default value
        scale.pack(fill='x', expand=True, padx=4, pady=4)

        var_name_label = tk.Label(frame, text=label, bg='black', fg='white', font=custom_font)
        var_name_label.pack(side='bottom')

        return scale

    # Creating scales for HSV values
    create_scale(root, "H_Low", 1, 179, 1, on_low_h_change)
    create_scale(root, "H_High", 1, 179, 179, on_high_h_change)
    create_scale(root, "S_Low", 1, 255, 1, on_low_s_change)
    create_scale(root, "S_High", 1, 255, 255, on_high_s_change)
    create_scale(root, "V_Low", 1, 255, 1, on_low_v_change)
    create_scale(root, "V_High", 1, 255, 255, on_high_v_change)

    # Spacer
    spacer = ttk.Frame(root, height=20, style="Outline.TFrame")
    spacer.pack(fill='x', expand=False, pady=10)

    # Creating scales for P-Control tuning (speed and angle sensitivity)
    create_scale(root, "Speed (%)", 0, 100, 0, on_speed_change)
    create_scale(root, "Angle Sensitivity (%)", 0, 100, 50, on_angsens_change)

    root.mainloop()

# [FUNCTION] Function to graph data to screen - threaded
def graph_data():
    fig, ax = plt.subplots()
    line, = ax.plot(range(hist_len), list(loc_history), label="Line Position")
    setpoint_line = ax.axhline(y=160, color='red', linestyle='--', label='Setpoint')

    # Set title and label
    ax.set_title("Plot of Current Line Position vs. Frame #")
    ax.set_xlabel("Frame Number")
    ax.set_ylabel("Pixels (px)")

    def update_plot(frame):
        line.set_ydata(loc_history)
        return line, setpoint_line

    # Set plot limits
    ax.set_ylim(0, 320)
    ax.set_xlim(0, hist_len - 1)

    # Add legend
    ax.legend()

    # Start animation
    ani = animation.FuncAnimation(fig, update_plot, interval=33, blit=True)
    
    # Show plot
    plt.show()

# [FUNCTION] Update the contour_center and contour_area each frame and display image - threaded
def update_contour(img):
    global contour_center
    global contour_area

    # Crop the image to the floor directly in front of the car
    image = rc_utils.crop(img, CROP_FLOOR[0], CROP_FLOOR[1])

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Find all of the contours of the saved color
        contours = rc_utils.find_contours(image, COLOR_THRESH[0], COLOR_THRESH[1])

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


# [FUNCTION] Start function isn't really needed here
def start():
    # Set initial driving speed and angle
    rc.drive.set_speed_angle(0, 0)

    # Thread it
    gui_thread = threading.Thread(target=create_gui)
    gui_thread.start()
    graph_thread = threading.Thread(target=graph_data)
    graph_thread.start()

    # Print start message
    print(
        ">> RACECAR Neo OneShot Demo - Line Follower with Live HSV Tuning\n"
        "\n"
        "Controls:\n"
        "   Right trigger = release RACECAR failsafe\n"
        "   A button = save tuned HSV value of line to system\n"
        "   B button = change between speed and angle modifier modes\n"
        "   X button = increase speed/angle depending on current mode\n"
        "   Y button = decrease speed/angle depending on current mode"
    )


# [FUNCTION] Tune HSV values to RACECAR camera while in main loop
def update():
    global COLOR_THRESH
    global CONTROL_PARAM
    global loc_history
    global speed
    global angle

    # Make manual mask for updating colors
    img = rc.camera.get_color_image()
    img = cv.resize(img, (320, 240)) # resize for speed reasons, high res not needed
    hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)  # Logitech camera returns BGR image
    hsv_low = np.array([H_low, S_low, V_low], np.uint8)
    hsv_high = np.array([H_high, S_high, V_high], np.uint8)
    mask = cv.inRange(hsv_image, hsv_low, hsv_high)
    cv.imshow('mask', mask)

    # Update contour function
    update_contour(img)

    # Define a basic p-controller. If contour is not found, keep last angle
    if contour_center is not None:
        setpoint = 160
        error = setpoint - contour_center[1]
        kp = -(2/setpoint) * CONTROL_PARAM[1]*2 # we want user defined scaled 0.5 -> 1
        angle = kp * error
        angle = rc_utils.clamp(angle, -1, 1) # clamp between -1 and 1

    # Define speed (between 0 and 1, user defined in UI)
    speed = CONTROL_PARAM[0]

    # Send speed and angle commands to RACECAR when trigger is pressed
    if rc.controller.get_trigger(rc.controller.Trigger.RIGHT) > 0.1:
        rc.drive.set_speed_angle(speed, angle)
        if len(loc_history) >= hist_len: # if size of history greater than spec 
            loc_history.pop(-1) # remove last element and proceed with append
        if contour_center is not None:
            loc_history.insert(0, contour_center[1])
        else:
            loc_history.insert(0, 0) # no detect
    else:
        rc.drive.set_speed_angle(0, 0)

    ######################
    # CONTROLLER OPTIONS #
    ######################

    # When A button is pressed, save HSV threshold & speed/angle values to memory
    if rc.controller.was_pressed(rc.controller.Button.A):
        print(f"HSV Threshold Saved!: ({H_low}, {S_low}, {V_low}), ({H_high}, {S_high}, {V_high})")
        COLOR_THRESH = ((H_low, S_low, V_low), (H_high, S_high, V_high))
        print(f"Speed/Angle Tuning Parameters Saved!: (tune_speed = {tune_speed}, tune_angle = {tune_angle})")
        CONTROL_PARAM = (tune_speed, tune_angle)

    # When B button is pressed, print SPEED and ANGLE to terminal window
    if rc.controller.was_pressed(rc.controller.Button.B):
        print(f"System Speed/Angle: Speed = {speed}, Angle = {angle}")
    


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
