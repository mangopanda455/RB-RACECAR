"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

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

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import numpy as np

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
grid = []
count = 0
########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global grid
    grid = [[7, 0], [3, 3], [1, 5], [1, 0], [1, 10], [1, 15], [1, 23], [1, 8], [2, 5], [2, 0], [2, 10], [2, 15], [2, 23], [2, 8], [3, 5], [3, 0], [3, 10], [3, 15], [3, 23], [3, 8], [4, 5], [4, 0], [4, 10], [4, 15], [4, 23], [5, 5], [5, 0], [5, 10], [5, 15], [5, 23], [5, 8], [6, 5], [6, 0], [6, 10], [6, 15], [6, 23], [6, 8], [0, 1], [0, 2], [0, 5], [0, 6], [0, 7], [0, 11], [0, 12], [0, 16], [0, 17], [0, 18], [0, 20], [0, 21], [0, 22], [7, 3], [7, 5], [7, 6], [7, 7], [7, 10], [7, 13], [7, 16], [7, 17], [7, 18], [7, 20], [7, 21], [7, 22], [4, 1], [4, 2], [4, 5], [4, 6], [4, 7], [4, 10], [4, 11], [4, 12], [4, 15], [4, 20], [4, 21], [4, 22], [4, 23], [1, 3], [2, 3], [5, 1], [6, 2], [7, 3], [1, 13], [2, 13], [3, 13], [5, 11], [6, 12], [7, 13]]
    

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    pass # Remove 'pass' and write your source code for the update() function here

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    global count
    global grid
    matrix = np.zeros((8, 24), dtype=np.uint8)
    for i in grid:
        if i[1] + count % 29 < 24:
            matrix[i[0], (i[1]+count)%29] = 1
        #matrix[i[0], (i[1]+ count)%29] = 1
    rc.display.set_matrix(matrix)
    count += 1


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.set_update_slow_time(0.2)
    rc.go()
