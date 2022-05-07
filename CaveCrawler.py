
from typing import Tuple, Iterator
from spike import PrimeHub, DistanceSensor, Motor, MotorPair
from math import radians, cos, sin, pi, copysign
from utime import sleep as wait_for_seconds

# some mapping code inspired from https://www.redblobgames.com/pathfinding/a-star/implementation.html
import gc
import uarray as array
# from micropython import const

# things that would usually be in a config file
wheel_base_cm = 12.2
wheel_diameter_cm = 5.6
# is this right offset?
l_ultrasonic_offset = r_ultrasonic_offset = 6
l_us_angle = - 90
r_us_angle = 90
l_motor = "E"
r_motor = "F"
l_ultrasonic = "A"
r_ultrasonic = "C"
turret_motor = "D"
# Python doesn't have built in constants, but we're going to agree to never
# change the following variables.
CONST_MAX_SPIKE_CM_DIST = 200
CONST_DETECTION_MAX_DIST_CM = 30
CONST_APPROACH_DIST_CM = 4
CONST_WAIT_TIME = 0.1
CONST_MAP_EDGE = 40
CONST_MAP_CELL_SIZE = 10
CONST_MOTOR_SPEED = 30
CONST_TURRET_SPEED = 20
CONST_STOP_ACTION = "hold"
CONST_TURRET_HOME = 0
CONST_TURRET_ANGLE_STEP = 6
CONST_TURRET_SWEEP_END = 180 + CONST_TURRET_HOME + CONST_TURRET_ANGLE_STEP + 1

Map_Location = Tuple[int, int]

class Map:
    def __init__(self, width: int, height: int) -> None:
        self.map = []
        self.width = width
        self.height = height
        for i in range(height):
            self.map.append(array.array(map_storage_vals["d_type"]))
            for j in range(width):
                self.map[i].append(map_storage_vals["new"])

    def add_to_map(self, id: Map_Location, value):
        (x, y) = id
        if self.in_bounds(id):
            self.map[x][y] = value

    def in_bounds(self, id: Map_Location) -> bool:
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def neighbors(self, id: Map_Location) -> Iterator[Map_Location]:
        (x, y) = id
        neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)] # E W N S
        if (x + y) % 2 == 0: neighbors.reverse() # S N W E
        results = filter(self.in_bounds, neighbors)
        # results = filter(self.passable, results)
        return results

    def cell(self, id: Map_Location):
        (x, y) = id
        if filter(self.in_bounds, id ):
            return map[x][y]
        else:
            raise Exception("Out of Bounds")


# initialize global objects
hub = PrimeHub()
motors = MotorPair(l_motor, r_motor)
turret = Motor(turret_motor)
motors.set_stop_action(CONST_STOP_ACTION)
turret.set_stop_action(CONST_STOP_ACTION)
turret.set_default_speed(CONST_TURRET_SPEED)
l_ultrasonic = DistanceSensor(l_ultrasonic)
r_ultrasonic = DistanceSensor(r_ultrasonic)

                    #type, unknown, empty, wall, robot
# map_storage_vals = ["b", '?', '-', '#', 'R']
# map_storage_vals = ["b", b'x/00', b'x\01', b'x\02', b'x\03']
# map_storage_vals = ["h", 0, -1, 1, 2] 

map_storage_vals = {
    "d_type" : 'h',
    "new" : 0,
    "empty" : -1,
    "obstacle" : 1,
    "robot" : 2
}

def drive_for_cm(cm):
    '''move the robot the specific number of cm forward'''
    rotation = 360 * cm / (pi * wheel_diameter_cm)
    motors.move_tank(rotation, unit="degrees", left_speed = CONST_MOTOR_SPEED, right_speed = CONST_MOTOR_SPEED)

def turn_for_degrees(degrees):
    '''rotate the robot the specified number of degrees'''
    sign = int(copysign(1, degrees))
    base_circumference = wheel_base_cm * pi
    cm = base_circumference * degrees / 360
    rotation = 360 * cm / (pi * wheel_diameter_cm)
    # I do not know why it makes me do it this way
    motors.move_tank(sign * rotation, unit="degrees", left_speed = sign * CONST_MOTOR_SPEED, right_speed = -sign * CONST_MOTOR_SPEED)

def take_turret_reading():
    '''return a pair of readings from the ultrasonic turret at the current location'''
    def take_distance_reading(ultrasonic):
        range = ultrasonic.get_distance_cm()
        value = map_storage_vals["obstacle"]
        if range is None:
            range = CONST_MAX_SPIKE_CM_DIST
            value = map_storage_vals["empty"]
        return (range, value)
    (range_l, value_l) = take_distance_reading(l_ultrasonic)
    wait_for_seconds(CONST_WAIT_TIME)
    (range_r, value_r) = take_distance_reading(r_ultrasonic)
    heading = turret.get_position()
    package = [(range_l + l_ultrasonic_offset, (heading + l_us_angle) % 360, value_l)]
    package.extend([(range_r + r_ultrasonic_offset, (heading + r_us_angle) % 360, value_r)])
    return package

def map_current_location(map, robot_location):
    '''take 360 degree ultrasonic readings from the current locations'''
    # list comprehension fails for some reason...
    # newmap = [[ 0 for g in range(2*200/CONST_MAP_CELL_SIZE)] for k in range(2*200/CONST_MAP_CELL_SIZE)]
    # this should do some position checking to ensure no errors with tangled wires
    turret.run_to_position(340, direction = "shortest path")
    turret.run_to_position(354, direction = "clockwise")
    readings = []
    for i in range(CONST_TURRET_HOME, CONST_TURRET_SWEEP_END, CONST_TURRET_ANGLE_STEP):
        wait_for_seconds(CONST_WAIT_TIME)
        readings.extend(take_turret_reading())
        turret.run_to_position(i, direction = "clockwise")
    turret.run_to_position(CONST_TURRET_HOME, direction = "counterclockwise")
    for reading in readings:
        (range_cm, heading, value) = reading
        # fill in detected cell
        loc_val = reading_to_relative_coordinate(reading, robot_location)
        add_coordinate_to_map(map, loc_val)
        (x, y, _ ) = loc_val
        # add neighbors as a buffer
        for cell in map.neighbors((x, y)):
            (t_x, t_y) = cell
            add_coordinate_to_map(map, (t_x, t_y, value))
        # fill in empty cells between
        for i in range(range_cm - CONST_MAP_CELL_SIZE, 0, -CONST_MAP_CELL_SIZE):
            add_coordinate_to_map(map, reading_to_relative_coordinate((i, heading, map_storage_vals["empty"]), robot_location))


def reading_to_relative_coordinate(reading, robot_location):
    '''convert a tuple of (range, heading) into cartesian coordinates'''
    # cos, sin
    # check that these are correctly offset
    (range, heading, value) = reading
    range = range / CONST_MAX_SPIKE_CM_DIST * CONST_MAP_CELL_SIZE
    x_coor = range*sin(radians(heading + robot_location[2]))
    # correct y_coordinate to N being 0 degrees for world coordinates
    y_coor = -range*cos(radians(heading + robot_location[2]))
    return (robot_location[0] + x_coor, robot_location[1] + y_coor, value)

def add_coordinate_to_map(map: Map, coordinate):
    '''edit a map coordinate to the specified value. Methodized to avoid mistakes.'''
    (x, y, value) = coordinate
    # only overwrite empty or new locations
    current_val = map.cell((x, y))
    #clunky
    if current_val is not None and current_val == map_storage_vals["empty"] or current_val == map_storage_vals["new"]:
        map.add_to_map((x,y), value)

def offset_map(map, robot_location, offset):
    return None

def rotate_map(map, robot_location, angle):
    return None

def printmap(map):
    output = ''
    for i in map:
        for j in range(len(i)):
            if i[j] == map_storage_vals["empty"]:
                output += '  '
            else:
                output += str(i[j]) + ' '
        output += '\n'
    print(output)

def test_loop():
    robot_location = [int(CONST_MAP_EDGE/2), int(CONST_MAP_EDGE/2), 0]
    newmap = Map(CONST_MAP_EDGE, CONST_MAP_EDGE)
    map_current_location(newmap, robot_location)
    newmap[18][18] = map_storage_vals["robot"]
    return newmap



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
    print(gc.mem_free())
    map = test_loop()
    print(gc.mem_free())
    # map2 = test_loop()
    # print(gc.mem_free())
    # map3 = test_loop()
    # print(gc.mem_free())
    printmap(map)
    # print()
    # printmap(map2)
    # map3 = 

    
    raise SystemExit

main()
