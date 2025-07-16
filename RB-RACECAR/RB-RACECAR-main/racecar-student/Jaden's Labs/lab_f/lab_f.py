import sys
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()
MIN_CONTOUR_AREA = 500
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))
ORANGE = ((1, 96, 131), (17, 251, 255))

speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

def update_contour():
    global contour_center
    global contour_area
    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found
        contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])
        largestc = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        if largestc is not None:
            contour_center = rc_utils.get_contour_center(largestc)
            contour_area = rc_utils.get_contour_area(largestc)
            rc_utils.draw_contour(image, largestc)
            rc_utils.draw_circle(image, contour_center)

def remap_range(val: float, old_min: float, old_max: float, new_min:float, new_max:float) -> float:
    old_range = old_max - old_min
    new_range = new_max - new_min
    return new_range * (float(val - old_min) / float(old_range)) + new_min
def start():
    global speed
    global angle
    speed = 0
    angle = 0
    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)

def update():
    global speed
    global angle
    update_contour()
    if contour_center is not None:
        setpoint = rc.camera.get_width()//2
        error = setpoint - contour_center[1]
        angle = max(min(3 * -remap_range(error, -320, 320, -1, 1), 1), -1)
    speed = 0.5
    angle *= 0.5

    rc.drive.set_speed_angle(speed, angle)

def update_slow():
    pass


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
