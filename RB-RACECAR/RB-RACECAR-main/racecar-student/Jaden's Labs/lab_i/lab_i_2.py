########################################################################################
# Imports
########################################################################################

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg") 
import matplotlib.pyplot as plt
from io import BytesIO
import cv2 
import heapq
import math

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here

speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
lidar_x = []
lidar_y = []
GRID_SIZE = 200          # n x n grid
GRID_WIDTH = 200         # width of the map in cm
CELL_SIZE = GRID_WIDTH / GRID_SIZE  # size of each cell in cm


########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    rc.drive.stop()

def update():
    global dstar
    (x, y) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    rc.drive.set_speed_angle(y, x)
    update_lidar_points()
    grid, img = generate_occupancy_grid()
    rc.display.show_color_image(img)

def update_lidar_points():
    global lidar_x, lidar_y
    scan = rc.lidar.get_samples() 
    angles = np.deg2rad(np.arange(0, 360, 0.5))
    distances = np.array(scan)
    valid = distances > 0
    distances = distances[valid]
    angles = angles[valid]
    # Polar to Cartesian conversion
    lidar_x = distances * np.sin(angles)
    lidar_y = distances * np.cos(angles)

def find_furthest_point():
    max_distance = -float('inf')
    max_index = -1

    for i in range(len(lidar_x)):
        x = lidar_x[i]
        y = lidar_y[i]
        # Only consider points in front of the car (y > 0)
        if y < 0:
           continue
        distance = np.hypot(x, y)
        if distance > max_distance and y > 0:
            max_distance = distance
            max_index = i
    #print(lidar_x[max_index])
    #print(lidar_y[max_index])
    return max_index, max_distance


def dilate_black_regions(img, kernel_size=6):
    # Create mask for black pixels (obstacles)
    black_mask = np.all(img == [0, 0, 0], axis=-1).astype(np.uint8)

    # Create 3x3 kernel
    kernel = np.ones((kernel_size, kernel_size), np.uint8)

    # Dilate the black mask
    dilated_mask = cv2.dilate(black_mask, kernel, iterations=7)

    # Apply the dilated mask back to the image
    # Any new pixels that were added by dilation and are currently white → turn them black
    new_black = (dilated_mask == 1) & (np.all(img == [255, 255, 255], axis=-1))
    img[new_black] = [0, 0, 0]

    return img


def generate_occupancy_grid():
    grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
    img = np.full((GRID_SIZE, GRID_SIZE, 3), 255, dtype=np.uint8)

    half_grid = GRID_SIZE // 2
    max_index, max_distance = find_furthest_point()
    print(max_index)
    print(max_distance)
    for idx, (x, y) in enumerate(zip(lidar_x, lidar_y)):
        i = int((y + GRID_WIDTH / 2) // CELL_SIZE)
        j = int((x + GRID_WIDTH / 2) // CELL_SIZE)
        i = GRID_SIZE - 1 - i  # Flip for image
        if 0 <= i < GRID_SIZE and 0 <= j < GRID_SIZE:
            if idx == max_index:
                grid[i, j] = 2
                img[i, j] = [0, 255, 0]  # Green = furthest point
                goal = (i, j)
            else:
                grid[i, j] = 1
                img[i, j] = [0, 0, 0]    # Black = obstacle
    img[half_grid, half_grid] = [128, 128, 0]
    start = (half_grid, half_grid)
    if 'goal' in locals():
        print(f'goal[0] = {goal[0]} and goal[1] = {goal[1]}')
        img[goal[0], goal[1]] = [0, 255, 0]
    if 'goal' in locals():  # only run if furthest point was valid
        path = a_star_path(grid, start, goal)
        for i, j in path:
            if grid[i, j] == 0:
                img[i, j] = [128, 0, 128]  # Purple path
    img = dilate_black_regions(img)    
    img = cv2.resize(img, (480, 360), interpolation=cv2.INTER_NEAREST)
    return grid, img


def a_star_path(grid, start, goal):
    def heuristic(a, b):
        x = abs(a[0]-b[0])
        y = abs(a[1]-b[1])
        return math.sqrt(x*x + y*y)
        #return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]  # 8-connectivity

    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))  # (f_score, g_score, position)
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, g, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)

            if not (0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE):
                continue  # out of bounds
            if grid[neighbor[0], neighbor[1]] == 1:
                continue  # obstacle

            tentative_g = g + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, tentative_g, neighbor))

    return []  # No path found



# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass  # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()