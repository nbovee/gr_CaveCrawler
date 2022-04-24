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
        self.start_ticks = 0

    def now(self):
        """Returns the time in seconds since the timer was last reset."""
        return ticks_diff(ticks_ms(), self.start_ticks) / 1000

    def reset(self):
        """Resets the timer."""
        self.start_ticks = ticks_ms()

# agnostic class for a given error funct
class Error:
    def __init__(self, func) -> None:
        self.error_func = func
        self.e, self.e_i, self.e_d = 0.
        self.target = None

    def set_target(self, target):
        self.target = target

    def get_target(self):
        return self.target

    def update_error(self):
        if self.target is not None:
            new_error = self.target - self.error_func()
            self.e_d = new_error - self.e
            self.e_i += new_error
            self.e = new_error

# PID for generic motor    
class PID_Motor(Motor):

    def __init__(self, port, k = 0.01, t_i = 1., t_d = 1., backlash = 0, power_ratio = 0.25) -> None:
        super().__init__(port)
        self.k = k
        self.t_i = t_i
        self.t_d = t_d
        self.backlash = backlash
        self.power_ratio = power_ratio
        self.__error = Error(self.get_degrees_counted)
        self.set_target(None)
        self.set_stop_action("brake")


    def set_target_angle_relative(self, target):
        current_degrees = self.__error.get_degrees_counted()
        result = current_degrees + target
        self.__error.set_target(result)

    def get_target_angle(self):
        return self.get_degrees_counted()

    def pid_calc(self):
        self.__error.update_error()
        new_power = self.k * ( self.__error.e + 1. / self.t_i* self.__error.e_i + self.t_d * self.__error.e_d)
        return new_power

    # quick way to make this work is recheck it at the start of each main() loop
    # the right way is to register them in a handler method
    def update_pid_outout(self):
        new_power = self.pid_calc()
        super.start_at_power(new_power)

    def start_at_power(self, power):
        raise Exception("Unsupported method.")

# handle drive motors    
class MotorPair:

    wheel_base_cm = 12.2
    wheel_diameter_cm = 5.6

    def __init__(self, l_motor, r_motor) -> None:
        self.l_motor = PID_Motor(l_motor)
        self.r_motor = PID_Motor(r_motor)
        self.l_target = 0 # self.l_motor.get_degrees_counted()
        self.r_target = 0 # self.r_motor.get_degrees_counted()
        self.l_motor.set_target_angle_relative(self.l_target)
        self.r_motor.set_target_angle_relative(self.r_target)


    def adjust_pid(self):
        self.l_motor.update_pid_outout()
        self.r_motor.update_pid_outout()

    def set_target_dist_cm(self, distance):
        rotation = 360 * distance / (pi * wheel_diameter_cm)
        self.l_motor.set_target_angle_relative(-rotation)
        self.r_motor.set_target_angle_relative(rotation)

    def set_target_turn_angle(self, angle):
        distance = wheel_base_cm * 2 * pi
        rotation = angle / 360 * distance / (pi * wheel_diameter_cm)
        self.l_motor.set_target_angle_relative(rotation)
        self.r_motor.set_target_angle_relative(rotation)

# better than nothing
class Odometer:

    def __init__(self, prime_hub) -> None:
        self.__hub = prime_hub
        self.x_vel = 0
        self.y_vel = 0
        self.x_loc = 0
        self.y_loc = 0
        self.yaw = self.__hub.motion.get_yaw_angle()
        self.__timer = Timer()
    
    def adjust_odometry(self):
        accel = self.hub.accelerometer()
        gyro = self.hub.gyroscope()
        now = self.__timer.now()**2
        self.__timer.reset()
        self.x_vel += accel[0]*now
        self.x_loc += self.x_vel*now
        self.y_vel += accel[1]*now
        self.y_loc += self.y_vel*now
        self.yaw = self.__hub.get_yaw_angle()

class Map:
    cell_size_cm = 2

    def __init__(self) -> None:
        
        

        
# things that would usually be in a config file
# lowest measured peak rpm was 151, we will target for 135rpm of that as our max output to allow for PID to adjust
# in practice we will aim for at most 50% of that value to avoid instability

l_ultrasonic_offset = r_ultrasonic_offset = 15.5/2
f_ultrasonic_offset = 8.5
l_motor_backlash_deg = 5
r_motor_backlash_deg = 6
l_motor = "E"
r_motor = "F"
f_ultrasonic = "A"
l_ultrasonic = "C"
r_ultrasonic = "D"

# initialize global objects
hub = PrimeHub()
odometer = Odometer(hub)
motors = MotorPair(l_motor, r_motor)
f_ultrasonic = DistanceSensor(f_ultrasonic)
l_ultrasonic = DistanceSensor(l_ultrasonic)
r_ultrasonic = DistanceSensor(r_ultrasonic)


def wheel_travel_distance(rotations):
    return rotations*pi*wheel_diameter_cm



# Python doesn't have built in constants, but we're going to agree to never
# change the following variables.
CONST_MAX_SPIKE_CM_DIST = 200
CONST_DETECTION_MAX_DIST_CM = 30
CONST_APPROACH_DIST_CM = 4
CONST_WAIT_TIME = 0.1
CONST_POW = 10



# -------------------- updateUltrasonic ----------------------------
# Assign Ultrasonic values in a single location for consistency. Must remain non-deterministic in calls.
def updateUltrasonic():
    myDistance = CONST_MAX_SPIKE_CM_DIST
    return CONST_ULTRASONIC.get_distance_cm() or myDistance


def main():
    motors.adjust_pid()
    searchAndApproach()
    raise SystemExit


main()
