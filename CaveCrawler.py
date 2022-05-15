"""
Author: Nicholas Bovee
This revision: 5/15/2022
Latest revision can be found at: https://github.com/nbovee/CaveCrawler

A video of the run from the accompanying log can be found here: https://www.youtube.com/watch?v=CwFstRNFZnY

We can see that the numerical implementation of SLAM with A* is extremely successful, with accurate mapping until environmental 
factors of slip and memory limitations on the Spike brick caused an error cascade. The errors appearance at the end of our trial 
is due to the restrictive angular offset search implemented. If SLAM methods assume less deflection than they actually 
encounter, then they are guaranteed to lose position over time. If we checked each potential offset up to say, 60 degrees, these 
issues are within the capability of the SLAM program to correct. However, each additional angle we inspect adds to our processing 
time, and standing still for 45 minutes filming was not going to happen at 3am. Secondary effects enhancing this error are the 
inconsistent friction of the carpet, and the inability of ultrasonic sensors to accurately detect surfaces they are not normal to.

Licensed Creative Commons Attribution-NonCommercial 4.0
https://creativecommons.org/licenses/by-nc/4.0

One of the best A* explanations I have found is at https://www.redblobgames.com/pathfinding/a-star/introduction.html
Minor portions of the code are structured after their implementation examples.
"""

import heapq
from spike import PrimeHub, DistanceSensor, Motor, MotorPair
from math import radians, degrees, cos, sin, pi, copysign, sqrt, atan2
from utime import sleep as wait_for_seconds
import gc
import uarray as array

# from micropython import const

# things that would usually be in a config file
wheel_base_cm = 12.2
wheel_diameter_cm = 5.6
# is this right offset?
l_ultrasonic_offset = r_ultrasonic_offset = 7
l_us_angle = -90
r_us_angle = 90
l_motor = "E"
r_motor = "F"
l_ultrasonic = "A"
r_ultrasonic = "C"
turret_motor = "D"
# Python doesn't have built in constants, but we're going to agree to never
# change the following variables.

# 63.5 x 152.5 cm2 area
CONST_DO_REVERSE_PATH = False
CONST_MOVE_STEPS = 8
CONST_DETECTION_MAX_DIST_CM = 30
CONST_APPROACH_DIST_CM = 4
CONST_WAIT_TIME = 0.1
CONST_MAP_EDGE_X = 16
CONST_MAP_EDGE_Y = 34
CONST_MAP_CELL_SIZE = 5
CONST_MOTOR_SPEED = 30
CONST_TURRET_SPEED = 20
CONST_STOP_ACTION = "hold"
CONST_USE_SHORTRANGE = True
CONST_MAX_SPIKE_CM_DIST = 70
if CONST_USE_SHORTRANGE:
    CONST_MAX_SPIKE_CM_DIST = 30
CONST_TURRET_HOME = 0
CONST_TURRET_ANGLE_STEP = 4
CONST_TURRET_SWEEP_END = 180 + CONST_TURRET_HOME + CONST_TURRET_ANGLE_STEP + 1
CONST_HOME = (CONST_MAP_EDGE_X - 4, CONST_MAP_EDGE_Y - 4)
CONST_GOAL = (8, 6)

# could rework this into a dict, or list of readings, for clear representation, but is arrays for memory conservation
# other future improvements would be to use a probability model for more accurate map representation long term
class Map:
    map_storage_vals = {"d_type": "h", "new": 0, "empty": -1, "obstacle": 1, "robot": 2}
    map_storage_keys = {0: "new", -1: "empty", 1: "obstacle", 2: "robot"}
    map_display_vals = {0: "?", -1: "-", 1: "#", 2: "R"}

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
                self.map[i].append(Map.map_storage_vals["new"])

    def nearest_wall(self, cell):
        """checks for the nearest wall in NSEW directions"""
        closest = 999
        (c_x, c_y) = cell
        for i in range(self.height):
            check = self.getitem(c_x, i)
            if check == self.map_storage_vals["obstacle"]:
                candidate = abs(c_y - i)
                if candidate < closest:
                    closest = candidate

        for j in range(self.width):
            check = self.getitem(j, c_y)
            if check == self.map_storage_vals["obstacle"]:
                candidate = abs(c_x - j)
                if candidate < closest:
                    closest = candidate
        return closest

    def cost(self, current, next):
        """return the cost of moving from current cell to next. Wall buffer is implemented here."""
        (c_x, c_y) = current
        (n_x, n_y) = next
        cost = abs(c_x - n_x) + abs(
            c_y - n_y
        )  # this should always be 1 in a grid, but just in case...
        next_val = self.getitem(*next)
        if next_val == self.map_storage_vals["obstacle"]:
            cost = 999  # in case filtering is not used
        wall_proximity = self.nearest_wall(next)
        cost += 10 / (
            int(wall_proximity**2) + 1
        )  # simple inverse function to avoid walls
        return int(cost)

    def add_to_map(self, id, value: str):
        (x, y) = id
        if self.in_bounds(id):
            self.map[int(y)][int(x)] = Map.map_storage_vals[value]

    def in_bounds(self, id) -> bool:
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def is_not_wall(self, id) -> bool:
        (x, y) = id
        return self.map[y][x] != self.map_storage_vals["obstacle"]

    def neighbors(self, id):
        (x, y) = id
        neighbors = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]  # E W N S
        if (x + y) % 2 == 0:
            # helps paths look a little nicer
            neighbors.reverse()  # S N W E
        results = filter(self.in_bounds, neighbors)
        results = filter(self.is_not_wall, results)
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

    def offset_location(self, x=None, y=None, yaw=None):
        if x is not None:
            self.robot_location_x += x
        if y is not None:
            self.robot_location_y += y
        if yaw is not None:
            self.robot_location_yaw += yaw

    def __str__(self) -> str:
        output = ""
        for i in range(self.height):
            for j in range(self.width):
                if j == self.robot_location_x and i == self.robot_location_y:
                    output += self.map_display_vals[Map.map_storage_vals["robot"]] + " "
                else:
                    output += self.map_display_vals[self.map[i][j]] + " "
            output += "\n"
        return output

    def getitem(self, x, y):
        if self.in_bounds((x, y)):
            return self.map[y][x]
        else:
            return None

    def setitem(self, x, y, value):
        self.map[y][x] = value

    def compare_equal_shape_map(self, map2):
        """maps to be compared should not have robot location marked"""
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
        """update itself with the non-new values from map2"""
        assert self.width == map2.width
        assert self.height == map2.height
        for i in range(self.height):
            for j in range(self.width):
                item = map2.getitem(j, i)
                if self.getitem(j, i) not in {
                    self.map_storage_vals["obstacle"],
                }:
                    self.map[i][j] = item


class Heap:
    def __init__(self, max_size=30) -> None:
        self.__heap = []
        self.max_size = max_size

    def push(self, element):
        # aggresively prune low likelihood cells to conserve RAM
        if bool(self.max_size) and len(self.__heap) > self.max_size:
            self.__heap.remove(max(self.__heap))
            # maintain heap structure
            heapq.heapify(self.__heap)
        heapq.heappush(self.__heap, element)

    def pop(self):
        return heapq.heappop(self.__heap)

    def has_elements(self):
        return bool(self.__heap)

    def __str__(self) -> str:
        return str(self.__heap)


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
    """move the robot the specific number of cm forward"""
    rotation = 360 * cm / (pi * wheel_diameter_cm)
    motors.move_tank(
        rotation,
        unit="degrees",
        left_speed=CONST_MOTOR_SPEED,
        right_speed=CONST_MOTOR_SPEED,
    )


def turn_for_degrees(degrees):
    """rotate the robot the specified number of degrees"""
    sign = int(copysign(1, degrees))
    base_circumference = wheel_base_cm * pi
    cm = base_circumference * degrees / 360
    rotation = 360 * cm / (pi * wheel_diameter_cm)
    # I do not know why it makes me do it this way
    motors.move_tank(
        sign * rotation,
        unit="degrees",
        left_speed=sign * CONST_MOTOR_SPEED,
        right_speed=-sign * CONST_MOTOR_SPEED,
    )


def take_turret_reading():
    """return a pair of readings from the ultrasonic turret at the current location"""

    def take_distance_reading(ultrasonic):
        range = ultrasonic.get_distance_cm(short_range=CONST_USE_SHORTRANGE)
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
    package.extend(
        [(range_r + r_ultrasonic_offset, (heading + r_us_angle) % 360, value_r)]
    )
    return package


def map_current_location(map):
    """take 360 degree ultrasonic readings from the current locations"""
    # list comprehension fails for some reason...
    # newmap = [[ 0 for g in range(2*200/CONST_MAP_CELL_SIZE)] for k in range(2*200/CONST_MAP_CELL_SIZE)]
    # this should do some position checking to ensure no errors with tangled wires
    print("Performing scan.", end="\n\n")
    turret.run_to_position(340, direction="shortest path")
    turret.run_to_position(354, direction="clockwise")
    readings = []
    for i in range(CONST_TURRET_HOME, CONST_TURRET_SWEEP_END, CONST_TURRET_ANGLE_STEP):
        wait_for_seconds(CONST_WAIT_TIME)
        readings.extend(take_turret_reading())
        turret.run_to_position(i, direction="clockwise")
    turret.run_to_position(CONST_TURRET_HOME, direction="counterclockwise")
    convert_readings_to_map(readings, map)


def convert_readings_to_map(readings, map):
    for reading in readings:
        (range_cm, heading, value) = reading
        # fill in detected cell
        loc_val = reading_to_coordinate(reading, map.current_location())
        add_coordinate_to_map(map, loc_val)
        (x, y, _) = loc_val
        # add neighbors as a buffer
        for cell in map.neighbors((x, y)):
            (t_x, t_y) = cell
            add_coordinate_to_map(map, (t_x, t_y, value))
        # fill in empty cells between
        for i in range(range_cm - CONST_MAP_CELL_SIZE, 0, -CONST_MAP_CELL_SIZE):
            add_coordinate_to_map(
                map,
                reading_to_coordinate((i, heading, "empty"), map.current_location()),
            )


def reading_to_coordinate(reading, robot_location):
    """convert a tuple of (range, heading) into cartesian coordinates"""
    # cos, sin
    # check that these are correctly offset
    (range, heading, value) = reading
    (x, y, yaw) = robot_location
    range = range / CONST_MAP_CELL_SIZE
    # range = range / CONST_MAX_SPIKE_CM_DIST * CONST_MAP_CELL_SIZE
    x_coor = range * sin(radians(heading + yaw))
    # correct y_coordinate to N being 0 degrees for world coordinates
    y_coor = -range * cos(radians(heading + yaw))
    return (round(x + x_coor), round(y + y_coor), value)


def add_coordinate_to_map(map: Map, coordinate):
    """edit a map coordinate to the specified value. Methodized for additional processing."""
    (x, y, value) = coordinate
    # round improves accuracy as long as we are checking our bounds internally in Map
    x = round(x)
    y = round(y)
    # only overwrite empty or new locations
    location_value = map.getitem(x, y)
    # only overwrite non-obstacle values
    if (
        location_value is not None
        and map.map_storage_vals["obstacle"] != location_value
    ):
        map.add_to_map((x, y), value)


def offset_map(map, offset):
    """Offset the map in the specified direction. Note that 1 cell of offset means -1 cells of offset on the robot."""
    (offset_x, offset_y) = offset
    if offset_x == 0 and offset_y == 0:
        return map  # early exit
    (x_loc, y_loc, yaw) = map.current_location()
    newmap = Map(map.width, map.height, (x_loc + offset_x, y_loc + offset_y, yaw))
    for i in range(map.height):
        for j in range(map.width):
            add_coordinate_to_map(
                newmap,
                (j + offset_x, i + offset_y, map.map_storage_keys[map.getitem(j, i)]),
            )
    return newmap


# could only write obstacle values, then reprocess empty spaces for higher accuracy and higher processing time
def rotate_map(map, angle):
    """Rotate the map in the specified direction. Note that 15 degrees of rotation means -15 degrees on the robot."""
    if angle == 0:
        return map  # early exit
    robot_location = map.current_location()
    (x_loc, y_loc, yaw) = robot_location
    newmap = Map(map.width, map.height, (x_loc, y_loc, yaw - angle))
    for i in range(newmap.height):
        for j in range(newmap.width):
            value = map.map_storage_keys[map.getitem(j, i)]
            x_dist = j - x_loc
            y_dist = i - y_loc
            # atan2 allows us to correctly identify the quadrant based on input signs
            cur_angle = atan2(y_dist, x_dist)
            new_angle = cur_angle + radians(angle)
            hyp = sqrt(x_dist**2 + y_dist**2)
            # these are validated transforms, if something in the map appearance is wrong, check the display methods first
            x_new = hyp * cos(new_angle)
            y_new = hyp * sin(new_angle)
            add_coordinate_to_map(
                newmap, (round(x_new) + x_loc, round(y_new) + y_loc, value)
            )
    return newmap


def drift_calculation(prev_map, cur_map):
    linear_offsets = [
        (0, 0),
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
    ]  # assuming diagonal drift is not happening
    angular_offsets = [
        0,
        -5,
        5,
        -10,
        10,
    ]  # major angular drift seems much harder to achieve
    scores = []
    print()
    for l in linear_offsets:
        for a in angular_offsets:
            out = "Comparing angular offset: " + str(a) + " & linear offset: " + str(l)
            print(out)
            score = prev_map.compare_equal_shape_map(
                rotate_map(offset_map(cur_map, l), a)
            )
            scores.append((score, l, a))
    print()
    # currently we avoid storing an ideal map to avoid RAM issues, and bite bullet to recalculate
    return max(scores)


def pathfind(map, goal):
    def heuristic(a, b):
        # basic manhattan distance
        # cannot *over*estimate the distance remaining, under is ok.
        (a_x, a_y) = a
        (b_x, b_y) = b
        return abs(a_x - b_x) + abs(a_y - b_y)

    # testing showed that we would run out of memory at around 40 elements in the heap. Targetting below as a precaution.
    print("Running A* Algorithm.")
    frontier = Heap(max_size=30)
    robot_loc = map.current_location()
    (x, y, a) = robot_loc  # for now we won't consider the facing angle in pathfinding
    start = (x, y)
    frontier.push((0, start))
    came_from = dict()
    cost_so_far = dict()
    came_from[start] = None
    cost_so_far[start] = 0
    while frontier.has_elements():
        current = frontier.pop()
        (score, (c_x, c_y)) = current
        current_id = (c_x, c_y)
        if (c_x, c_y) == goal:
            break
        # walls are never passable, and will clog our heap
        for next in filter(map.is_not_wall, map.neighbors(current_id)):
            # partial_cost = map.cost(current_id, next)
            new_cost = cost_so_far[current_id] + map.cost(current_id, next)
            # excluding the second term below implements MPP by assuming the first encounter is the ideal.
            if next not in cost_so_far:  # or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.push((priority, next))
                came_from[next] = current_id

    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    # cleaner processing if we do not reverse the path
    # path.reverse()
    return path


def movement_loop(map, goal=CONST_GOAL):
    # identify path
    path = pathfind(map, goal)
    print("A* complete, selected the following path:")
    print(path, end="\n\n")
    # identify movements
    current_cell = path.pop()
    direction_dict = {(0, -1): "N", (1, 0): "E", (0, 1): "S", (-1, 0): "W"}
    angle_dict = {0: "N", 90: "E", 180: "S", 270: "W"}
    (_, _, angle) = map.current_location()
    current_direction = angle_dict[angle]
    print("Current robot heading is " + current_direction + ".")
    steps = min(CONST_MOVE_STEPS, len(path))
    print("Executing " + str(steps) + " motions.")
    for i in range(steps):
        print("Motion " + str(i) + ":")
        next_cell = path.pop()  # pop with no index pulls from list[-1]
        (c_x, c_y) = current_cell
        (n_x, n_y) = next_cell
        next_direction = direction_dict[
            (n_x - c_x), (n_y - c_y)
        ]  # this functions on world coordinates
        # turn to face
        turn_amount = 0
        drive_amount = CONST_MAP_CELL_SIZE
        if next_direction != current_direction:
            # either we turn left 90, turn right 90, or turn 180
            dir_list = ["N", "E", "S", "W"]  # dict values wont be in this order
            turn_code = dir_list.index(next_direction) - dir_list.index(
                current_direction
            )
            if turn_code in {1, -3}:
                turn_amount = 90
            elif turn_code in {-1, 3}:
                turn_amount = -90
            else:
                turn_amount = 180
            # execute movements, set to coast for smoothness
            print("\tTurning to face " + next_direction + ".")
        if i == 0:
            motors.set_stop_action("coast")
        if turn_amount != 0:
            turn_for_degrees(turn_amount)
        if i == steps - 1:
            motors.set_stop_action(CONST_STOP_ACTION)
        print("\tDriving for: " + str(drive_amount) + " cm.")
        drive_for_cm(drive_amount)
        current_cell = next_cell
        current_direction = next_direction
    # create assumed robot location from movements
    # we ignore the IMU and assume heading based on grid motions
    yaw_dict = {"N": 0, "E": 90, "S": 180, "W": 270}
    robot_yaw = yaw_dict[current_direction]
    # robot location is assumed to be the last cell of the path
    (robot_x, robot_y) = current_cell
    # create second map with assumed robot location
    newmap = Map(CONST_MAP_EDGE_X, CONST_MAP_EDGE_Y, (robot_x, robot_y, robot_yaw))
    map_current_location(newmap)
    print("Map after motions:")
    print(newmap)
    # compare
    ideal = drift_calculation(map, newmap)
    # combine and inherit the best scored transformation as accurate.
    (_, l, a) = ideal
    print(
        "The best transformation is an offset of: "
        + str(l)
        + " and a rotation of: "
        + str(a)
        + " degrees.",
        end="\n\n",
    )
    print("Combining original map with new values.")
    map.combine_maps(rotate_map(offset_map(newmap, l), a))
    # update location (yaw will be ideal momentarily)
    (lx, ly) = l
    new_x = newmap.robot_location_x - lx
    new_y = newmap.robot_location_y - ly
    # the robot is displaced in the opposite direction of map movement
    map.update_location(
        x=new_x,
        y=new_y,
        yaw=newmap.robot_location_yaw,
    )
    del newmap  # ensure memory is released
    # compensate yaw
    turn_for_degrees(-a)
    print("Combined Map:")
    print(map)
    # return whether goal was reached
    return CONST_GOAL == (new_x, new_y)


def main():
    global hub
    # assume the robot is placed accurately at the start location facing north
    hub.motion_sensor.reset_yaw_angle()
    print("Starting CaveCrawler.", end="\n\n")
    # initialize
    (h_x, h_y) = CONST_HOME
    robot_location = (h_x, h_y, hub.motion_sensor.get_yaw_angle())
    map = Map(CONST_MAP_EDGE_X, CONST_MAP_EDGE_Y, robot_location)
    map_current_location(map)
    print("First Map:")
    print(map)
    goal_found = False
    while not goal_found:
        goal_found = movement_loop(map)
    # do the reverse path if we want
    if CONST_DO_REVERSE_PATH:
        goal_found = False
    while not goal_found:
        goal_found = movement_loop(map, goal=CONST_HOME)
    raise SystemExit


main()
