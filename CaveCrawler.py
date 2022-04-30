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
        self.start_ticks = ticks_ms() - 1

    def now(self):
        """Returns the time in seconds since the timer was last reset."""
        return ticks_diff(ticks_ms(), self.start_ticks) / 1000

    def reset(self):
        """Resets the timer."""
        self.start_ticks = ticks_ms()

# agnostic class for a given error funct
class Error:
    def __init__(self, obj, func) -> None:
        self.err_obj = obj
        self.error_func = func
        self.e = self.e_i = self.e_d = 0.
        self.target = None
        self.timer = None

    def set_target(self, target):
        self.target = target

    def get_target(self):
        return self.target

    def update_error(self):
        if self.timer is None:
            self.timer = Timer()
        if self.target is not None:
            new_error = self.target - getattr(self.err_obj, self.error_func)()
            if abs(new_error) < 3:
                new_error = 0 # inaccurate motor compensation & motor control inaccuracy
            t_d = self.timer.now()
            self.timer.reset()
            self.e_d = (new_error - self.e) / t_d
            self.e_i += new_error * t_d
            self.e = new_error
            self.timer.reset()

# PID for generic motor    
class PID_Motor(Motor):

    def __init__(self, port, k = 50, k_i = 10, k_d =  4) -> None:
        super().__init__(port)
        self.k = k/10**2
        self.k_i = k_i/10**2
        self.k_d = k_d/10**2
        self.__error = Error(self, "get_degrees_counted")
        self.__error.set_target(None)
        self.set_stop_action("brake")


    def set_target_angle_relative(self, target):
        current_degrees = self.get_degrees_counted()
        result = current_degrees + target
        self.__error.set_target(result)

    def get_target_angle(self):
        return self.__error.get_target()

    def pid_calc(self):
        self.__error.update_error()
        # this is a better way to do it that we will ignore
        # new_power = self.k * ( self.__error.e + 1. / self.t_i* self.__error.e_i + self.t_d * self.__error.e_d)
        # this is the commonly implemented form
        new_power = self.k * self.__error.e + self.k_i * self.__error.e_i + self.k_d * self.__error.e_d
        return new_power

    # quick way to make this work is recheck it at the start of each main() loop
    # the right way is to register them in a handler method
    def update_pid_outout(self):
        new_power = self.pid_calc()
        self.next_power = int(new_power)
        


# handle drive motors    
class MotorPair:
 # maybe these should syncronize instead? Error based off the delta between position? percentile complete of the target move?
 # give the true offset if moving in a straing line, or the inverse if doing a point turn?
    wheel_base_cm = 12.2
    wheel_diameter_cm = 5.6

    def __init__(self, l_motor, r_motor) -> None:
        self.l_motor = PID_Motor(l_motor, k = 40)
        self.r_motor = PID_Motor(r_motor)
        self.l_target = 0 
        self.r_target = 0 
        self.l_motor.set_target_angle_relative(self.l_target)
        self.r_motor.set_target_angle_relative(self.r_target)


    def adjust_pid(self):
        self.l_motor.update_pid_outout()
        self.r_motor.update_pid_outout()
        #start separately to sync
        self.l_motor.start(self.l_motor.next_power)
        self.r_motor.start(self.r_motor.next_power)

    def set_target_dist_cm(self, distance):
        rotation = 360 * distance / (pi * MotorPair.wheel_diameter_cm)
        self.l_motor.set_target_angle_relative(-rotation)
        self.r_motor.set_target_angle_relative(rotation)

    def set_target_turn_angle(self, angle):
        distance = MotorPair.wheel_base_cm * 2 * pi
        rotation = angle / 360 * distance / (pi * MotorPair.wheel_diameter_cm)
        self.l_motor.set_target_angle_relative(rotation)
        self.r_motor.set_target_angle_relative(rotation)

# # better than nothing
# class Odometer:

#     def __init__(self, prime_hub) -> None:
#         self.__hub = prime_hub
#         self.x_acc = 0
#         self.y_acc = 0
#         self.x_vel = 0
#         self.y_vel = 0
#         self.x_loc = 0
#         self.y_loc = 0
#         # self.yaw = self.__hub.motion.get_yaw_angle()
#         self.__timer = Timer()
    
#     def adjust_odometry(self):
#         accel = self.hub.accelerometer()
#         gyro = self.hub.gyroscope()
#         now = self.__timer.now()**2
#         self.__timer.reset()
#         self.x_acc = accel[0]
#         self.y_acc = accel[1]
#         self.x_vel += self.x_acc*now
#         self.x_loc += self.x_vel*now
#         self.y_vel += self.y_acc*now
#         self.y_loc += self.y_vel*now
#         # self.yaw = self.__hub.get_yaw_angle()

    # def __str__(self):
    #     return f"({self.x_loc}, {self.y_loc})"

# class Map:
#     cell_size_cm = 2

#     def __init__(self) -> None:
        
        


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
# odometer = Odometer(hub)
motors = MotorPair(l_motor, r_motor)
# f_ultrasonic = DistanceSensor(f_ultrasonic)
# l_ultrasonic = DistanceSensor(l_ultrasonic)
# r_ultrasonic = DistanceSensor(r_ultrasonic)


# Python doesn't have built in constants, but we're going to agree to never
# change the following variables.
CONST_MAX_SPIKE_CM_DIST = 200
CONST_DETECTION_MAX_DIST_CM = 30
CONST_APPROACH_DIST_CM = 4
CONST_WAIT_TIME = 0.1
CONST_POW = 10


# -------------------- updateUltrasonic ----------------------------
# Assign Ultrasonic values in a single location for consistency. Must remain non-deterministic in calls.
# def updateUltrasonic():
#     myDistance = CONST_MAX_SPIKE_CM_DIST
#     return CONST_ULTRASONIC.get_distance_cm() or myDistance


def main():
    # motors.set_target_dist_cm(10)
    motors.set_target_turn_angle(90)
    print(motors.l_motor.get_target_angle())
    print(motors.r_motor.get_target_angle())
    wait_for_seconds(5)
    checker = Timer()
    while(checker.now() < 5):
        motors.adjust_pid()
        print(motors.l_motor.__error.e, end="\t")
        print(motors.l_motor.__error.e_i, end="\t")
        print(motors.l_motor.__error.e_d, end="\t\t")
        print(motors.r_motor.__error.e, end="\t")
        print(motors.r_motor.__error.e_i, end="\t")
        print(motors.r_motor.__error.e_d)
        # odometer.adjust_odometry()
        # print(odometer) # this is probably also not needed
    raise SystemExit


main()
