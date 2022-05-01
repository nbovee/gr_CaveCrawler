from multiprocessing.connection import wait
from operator import add
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from math import *
from utime import sleep as wait_for_seconds
from utime import ticks_diff, ticks_ms
from spike.control import wait_until


# portions from https://gist.github.com/dlech/fa48f9b2a3a661c79c2c5880684b63ae

# spike.control.Timer doesn't allow decimal points
class Timer():
    """Replacement Timer class that allows decimal points so we can measure times of less than one second."""
    def __init__(self):
        self.start_ticks = ticks_ms() - 1 # avoid divide by zeros later on

    def now(self):
        """Returns the time in seconds since the timer was last reset."""
        return ticks_diff(ticks_ms(), self.start_ticks) / 1000

    def reset(self):
        """Resets the timer."""
        self.start_ticks = ticks_ms()

# things that would usually be in a config file
wheel_base_cm = 12.2
wheel_diameter_cm = 5.6
# update this
l_ultrasonic_offset = r_ultrasonic_offset = 15.5/2
# f_ultrasonic_offset = 8.5
# l_motor_backlash_deg = 5
# r_motor_backlash_deg = 6
l_motor = "E"
r_motor = "F"
# f_ultrasonic = "A"
l_ultrasonic = "A"
r_ultrasonic = "C"
turret_motor = "D"
# Python doesn't have built in constants, but we're going to agree to never
# change the following variables.
CONST_MAX_SPIKE_CM_DIST = 200
CONST_DETECTION_MAX_DIST_CM = 30
CONST_APPROACH_DIST_CM = 4
CONST_WAIT_TIME = 0.1

CONST_MAP_CELL_SIZE = 2

# initialize global objects
hub = PrimeHub()
motors = MotorPair(l_motor, r_motor)
turret = Motor(turret_motor)
motors.set_stop_action("hold")
turret.set_stop_action("hold")
# f_ultrasonic = DistanceSensor(f_ultrasonic)
l_ultrasonic = DistanceSensor(l_ultrasonic)
r_ultrasonic = DistanceSensor(r_ultrasonic)


def drive_for_cm(cm):
    '''move the robot the specific number of cm forward'''
    rotation = 360 * cm / (pi * wheel_diameter_cm)
    motors.move_tank(rotation, unit="degrees", left_speed = 30, right_speed = 30)

def turn_for_degrees(degrees):
    '''rotate the robot the specified number of degrees'''
    sign = int(copysign(1, degrees))
    base_circumference = wheel_base_cm * pi
    cm = base_circumference * degrees / 360
    rotation = 360 * cm / (pi * wheel_diameter_cm)
    # I do not know why it makes me do it this way
    motors.move_tank(sign * rotation, unit="degrees", left_speed = sign * 30, right_speed = -sign *30)

def take_turret_reading():
    '''return a pair of readings from the ultrasonic turret at the current location'''
    def take_distance_reading(ultrasonic):
        range = ultrasonic.range()
        value = 1
        if range is None:
            range = CONST_MAX_SPIKE_CM_DIST
            value = -1
        return (range, value)

    (range_l, value_l) = take_distance_reading(l_ultrasonic)
    wait_for_seconds(CONST_WAIT_TIME)
    (range_r, value_r) = take_distance_reading(r_ultrasonic)
    wait_for_seconds(CONST_WAIT_TIME)
    heading = turret.get_angle()
    return [(range_l + l_ultrasonic_offset, heading - 90, value_l), (range_r + r_ultrasonic_offset, heading + 90, value_r)]


def map_current_location(robot_location):
    '''take 360 degree ultrasonic readings from the current locations'''
    # bias the sensor to avoid backlash
    turret.move_to(-5)
    readings = []
    for i in range(0,180,5):
        turret.move_to(i)
        wait_for_seconds(CONST_WAIT_TIME)
        readings.extend(take_turret_reading())
    turret.move_to(-5)
    for reading in readings:
        (range, heading, value) = reading
        # fill in detected cell
        add_coordinate_to_map(map, reading_to_relative_coordinate(reading, robot_location))
        # fill in empty cells between
        for i in range(range, 0, CONST_MAP_CELL_SIZE):
            add_coordinate_to_map(map, reading_to_relative_coordinate((i, heading, -1), robot_location))
    

def reading_to_relative_coordinate(reading, robot_location):
    '''convert a tuple of (range, heading) into cartesian coordinates'''
    # cos, sin
    # check that these are correctly offset
    (range, heading, value) = reading
    x_coor = range*sin(radians(heading + robot_location.yaw))
    y_coor = range*cos(radians(heading + robot_location.yaw))
    return (robot_location.x + x_coor, robot_location.y + y_coor, value)

def add_coordinate_to_map(map, coordinate):
    '''edit a map coordinate to the specified value. Methodized to avoid mistakes.'''
    (x, y, value) = coordinate
    map[x][y] = value

def offset_map(map, robot_location, offset):
    return None

def rotate_map(map, robot_location, angle):
    return None




def main():
    # can we move at things other than right angles?

    # loop until feature overlap
        # move small amount
        # gather data

    # calculate distorted maps
    # multiply pairwise

    # select and adjust post

    #repeat?

    # A* search main map

    # drive_for_cm(5)
    drive_for_cm(2)
    wait_for_seconds(CONST_WAIT_TIME)
    drive_for_cm(2)
    wait_for_seconds(CONST_WAIT_TIME)
    drive_for_cm(2)
    wait_for_seconds(CONST_WAIT_TIME)
    drive_for_cm(2)
    # turn_for_degrees(-5)
    raise SystemExit

main()
