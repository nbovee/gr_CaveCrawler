from spike import PrimeHub, DistanceSensor, Motor, MotorPair
from math import radians, degrees, cos, sin, pi, copysign, sqrt, atan2
from utime import sleep as wait_for_seconds

# some mapping code inspired from https://www.redblobgames.com/pathfinding/a-star/implementation.html
import gc
import uarray as array
# from micropython import const

# things that would usually be in a config file
wheel_base_cm = 12.2
wheel_diameter_cm = 5.6
# is this right offset?
l_ultrasonic_offset = r_ultrasonic_offset = 7
l_us_angle = - 90
r_us_angle = 90
l_motor = "E"
r_motor = "F"
l_ultrasonic = "A"
r_ultrasonic = "C"
turret_motor = "D"
# Python doesn't have built in constants, but we're going to agree to never
# change the following variables.

#63.5 x 152.5 cm2 area
CONST_DETECTION_MAX_DIST_CM = 30
CONST_APPROACH_DIST_CM = 4
CONST_WAIT_TIME = 0.1
CONST_MAP_EDGE_X = 26
CONST_MAP_EDGE_Y = 56
CONST_MAP_CELL_SIZE = 3 #12 # approx wheel base size so pathing is already at robot width
CONST_MOTOR_SPEED = 30
CONST_TURRET_SPEED = 20
CONST_STOP_ACTION = "hold"
CONST_USE_SHORTRANGE = False
CONST_MAX_SPIKE_CM_DIST = 70
if CONST_USE_SHORTRANGE:
    CONST_MAX_SPIKE_CM_DIST = 30
CONST_TURRET_HOME = 0
CONST_TURRET_ANGLE_STEP = 4
CONST_TURRET_SWEEP_END = 180 + CONST_TURRET_HOME + CONST_TURRET_ANGLE_STEP + 1

# synthetic_reading_list = [(34, 262, 'obstacle'), (21, 82, 'obstacle'), (34, 270, 'obstacle'), (21, 90, 'obstacle'), (35, 276, 'obstacle'), (36, 96, 'obstacle'), (36, 280, 'obstacle'), (34, 100, 'obstacle'), (40, 285, 'obstacle'), (37, 105, 'obstacle'), (40, 291, 'obstacle'), (206, 111, 'empty'), (38, 299, 'obstacle'), (206, 119, 'empty'), (39, 303, 'obstacle'), (206, 123, 'empty'), (46, 312, 'obstacle'), (206, 132, 'empty'), (46, 316, 'obstacle'), (206, 136, 'empty'), (46, 322, 'obstacle'), (206, 142, 'empty'), (45, 329, 'obstacle'), (206, 149, 'empty'), (47, 335, 'obstacle'), (95, 155, 'obstacle'), (38, 341, 'obstacle'), (95, 161, 'obstacle'), (41, 346, 'obstacle'), (94, 166, 'obstacle'), (34, 353, 'obstacle'), (95, 173, 'obstacle'), (35, 0, 'obstacle'), (94, 180, 'obstacle'), (34, 3, 'obstacle'), (206, 183, 'empty'), (39, 10, 'obstacle'), (206, 190, 'empty'), (34, 17, 'obstacle'), (206, 197, 'empty'), (41, 21, 'obstacle'), (206, 201, 'empty'), (39, 28, 'obstacle'), (206, 208, 'empty'), (39, 33, 'obstacle'), (206, 213, 'empty'), (39, 40, 'obstacle'), (206, 220, 'empty'), (39, 47, 'obstacle'), (206, 227, 'empty'), (39, 51, 'obstacle'), (206, 231, 'empty'), (20, 58, 'obstacle'), (206, 238, 'empty'), (20, 63, 'obstacle'), (206, 243, 'empty'), (20, 70, 'obstacle'), (206, 250, 'empty'), (21, 76, 'obstacle'), (206, 256, 'empty'), (21, 82, 'obstacle'), (206, 262, 'empty'), (21, 87, 'obstacle'), (206, 267, 'empty')]

#could rework this into a dict, or list of readings, for clear representation, but is arrays for memory conservation
class Map:
    map_storage_vals = {
        "d_type" : 'h',
        "new" : 0,
        "empty" : -1,
        "obstacle" : 1,
        "robot" : 2
    }
    map_storage_keys = {
        0 : "new",
        -1 : "empty",
        1: "obstacle",
        2 : "robot"
    }
    map_display_vals = {
        0 : '?',
        -1 : '-',
        1 : '#',
        2 : 'R'
    }
    def __init__(self, width: int, height: int, robot_location) -> None:
        self.map = []
        self.width = width
        self.height = height
        (x, y, yaw) = robot_location
        self.robot_location_x = x
        self.robot_location_y = y
        self.robot_location_yaw = yaw

        # note our origin is the top left corner of the map
        # could be single array with parseing but this is easier
        for i in range(height):
            self.map.append(array.array(Map.map_storage_vals["d_type"]))
            for j in range(width):
                # if i == 0 or i == height -1 or j == 0 or j == width -1:
                #     self.map[i].append(Map.map_storage_vals["obstacle"])
                # else:
                    self.map[i].append(Map.map_storage_vals["new"])

    def add_to_map(self, id, value : str):
        (x ,y) = id
        if self.in_bounds(id):
            self.map[int(y)][int(x)] = Map.map_storage_vals[value]

    def in_bounds(self, id) -> bool:
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def neighbors(self, id):
        (x, y) = id
        neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)] # E W N S
        if (x + y) % 2 == 0: neighbors.reverse() # S N W E
        results = filter(self.in_bounds, neighbors)
        # results = filter(self.passable, results)
        return results

    def current_location(self):
        return (self.robot_location_x, self.robot_location_y, self.robot_location_yaw)

    def update_location(self, x=None, y=None, yaw=None):
        if x is not None:
            self.robot_location_x = x
        if y is not None:
            self.robot_location_y = y
        if yaw is not None:
            self.robot_location_yaw = yaw

    def expose(self):
        '''expose the internal map for first pass monolith programming'''
        return self.map

    def __str__(self) -> str:
        output = ''
        for i in range(self.height):
            for j in range(self.width):
                if j == self.robot_location_x and i == self.robot_location_y:
                    output += self.map_display_vals[Map.map_storage_vals["robot"]] + ' '    
                else:
                    output += self.map_display_vals[self.map[i][j]] + ' '
            output += '\n'
        return output

    def getitem(self, x, y):
        if self.in_bounds((x, y)):
            return self.map[y][x]
        else:
            return None
    
    def setitem(self, x, y, value):
        self.map[y][x] = value

    def compare_equal_shape_map(self, map2):
        '''maps to be compared should not have robot location marked'''
        assert self.width == map2.width
        assert self.height == map2.height
        score = 0
        for i in range(self.height):
            for j in range(self.width):
                score += self.getitem(j, i) * map2.getitem(j, i) 
        # check width and height
        # multiple cellwise & return sum
        return score

    def combine_maps(self, map2):
        pass


# initialize global objects
hub = PrimeHub()
motors = MotorPair(l_motor, r_motor)
turret = Motor(turret_motor)
motors.set_stop_action(CONST_STOP_ACTION)
turret.set_stop_action(CONST_STOP_ACTION)
turret.set_default_speed(CONST_TURRET_SPEED)
l_ultrasonic = DistanceSensor(l_ultrasonic)
r_ultrasonic = DistanceSensor(r_ultrasonic)


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
        range = ultrasonic.get_distance_cm(short_range = CONST_USE_SHORTRANGE)
        value = "obstacle"
        if range is None or range > CONST_MAX_SPIKE_CM_DIST:
            range = CONST_MAX_SPIKE_CM_DIST
            value = "empty"

        return (range, value)
    (range_l, value_l) = take_distance_reading(l_ultrasonic)
    wait_for_seconds(CONST_WAIT_TIME)
    (range_r, value_r) = take_distance_reading(r_ultrasonic)
    heading = turret.get_position()
    package = [(range_l + l_ultrasonic_offset, (heading + l_us_angle) % 360, value_l)]
    package.extend([(range_r + r_ultrasonic_offset, (heading + r_us_angle) % 360, value_r)])
    return package

def map_current_location(map):
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
    convert_readings_to_map(readings, map)

def convert_readings_to_map(readings, map):
    for reading in readings:
        (range_cm, heading, value) = reading
        # fill in detected cell
        loc_val = reading_to_coordinate(reading, map.current_location())
        add_coordinate_to_map(map, loc_val)
        (x, y, _ ) = loc_val
        # add neighbors as a buffer
        for cell in map.neighbors((x, y)):
            (t_x, t_y) = cell
            add_coordinate_to_map(map, (t_x, t_y, value))
        # fill in empty cells between
        for i in range(range_cm - CONST_MAP_CELL_SIZE, 0, -CONST_MAP_CELL_SIZE):
            add_coordinate_to_map(map, reading_to_coordinate((i, heading, "empty"), map.current_location()))


def reading_to_coordinate(reading, robot_location):
    '''convert a tuple of (range, heading) into cartesian coordinates'''
    # cos, sin
    # check that these are correctly offset
    (range, heading, value) = reading
    (x, y, yaw) = robot_location
    range = range / CONST_MAP_CELL_SIZE
    # range = range / CONST_MAX_SPIKE_CM_DIST * CONST_MAP_CELL_SIZE
    x_coor = range*sin(radians(heading + yaw))
    # correct y_coordinate to N being 0 degrees for world coordinates
    y_coor = -range*cos(radians(heading + yaw))
    return (round(x + x_coor), round(y + y_coor), value)

def add_coordinate_to_map(map: Map, coordinate):
    '''edit a map coordinate to the specified value. Methodized to avoid mistakes.'''
    (x, y, value) = coordinate
    x = round(x)
    y = round(y)
    # only overwrite empty or new locations
    location_value = map.getitem(x, y)
        # only overwrite non-obstacle values
    if location_value is not None and map.map_storage_vals["obstacle"] != location_value:
        map.add_to_map((x,y), value)



def offset_map(map, offset):
    (offset_x, offset_y) = offset
    if offset_x == 0 and offset_y ==0:
        return map # early exit
    (x_loc, y_loc, yaw) = map.current_location()
    newmap = Map(map.width, map.height, (x_loc + offset_x, y_loc + offset_y, yaw))
    for i in range(map.height):
        for j in range(map.width):
            add_coordinate_to_map(newmap,(j + offset_x, i + offset_y, map.map_storage_keys[map.getitem(j,i)]))
    return newmap

# could only write obstacle values, then reprocess empty spaces for higher accuracy
def rotate_map(map, angle):
    if angle == 0:
        return map # exit exit
    robot_location = map.current_location()
    (x_loc, y_loc, yaw ) = robot_location
    newmap = Map(map.width, map.height, robot_location)
    for i in range(newmap.height):
        for j in range(newmap.width):
            value = map.map_storage_keys[map.getitem(j, i)]
            x_dist = j - x_loc
            y_dist = i - y_loc
            # atan2 allows us to correctly identify the quadrant based on input signs
            cur_angle = atan2(y_dist, x_dist)
            new_angle = cur_angle + radians(angle)
            hyp = sqrt(x_dist ** 2 + y_dist ** 2)
            x_new = hyp * cos(new_angle)
            y_new = hyp * sin(new_angle)
            add_coordinate_to_map(newmap, (round(x_new) + x_loc, round(y_new) + y_loc, value))
    return newmap

def drift_calculation(prev_map, cur_map):
    linear_offsets = [(0,0), (-1, 0), (1, 0), (0,-1), (0, 1)] # forward and back drift seems tough, but must be considered
    angular_offsets = [0, -5, 5] # major angular drift seems much harder to achieve
    scores = []
    for l in linear_offsets:
        for a in angular_offsets:
            score = prev_map.compare_equal_shape_map(rotate_map(offset_map(cur_map, l), a))
            scores.append((score, l, a))
    print(scores)
    return max(scores)

def test_loop():
    global hub
    
    robot_location = (4, CONST_MAP_EDGE_Y - 6, hub.motion_sensor.get_yaw_angle())
    map = Map(CONST_MAP_EDGE_X, CONST_MAP_EDGE_Y, robot_location)
    # print("Before Scanning:")
    # print(map)
    map_current_location(map)
    # print("After Mapping:")
    # print(map)
    turn_for_degrees(90)
    drive_for_cm(5 * CONST_MAP_CELL_SIZE)
    robot_location_2 = (4 + 5 , CONST_MAP_EDGE_Y -6, robot_location[2]+90)
    newmap = Map(CONST_MAP_EDGE_X, CONST_MAP_EDGE_Y, robot_location_2)
    map_current_location(newmap)
    # print("Second Location:")
    # print(newmap)
    ideal = drift_calculation(map, newmap)
    print(ideal)

    return True



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
    
    raise SystemExit

main()
