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


class Error:
    def __init__(self, func) -> None:
        self.error_func = func
        self.e, self.e_i, self.e_d = 0.
        self.target = None

    def set_target(self, target):
        self.target = target

    def update_error(self):
        if self.target is not None:
            new_error = self.target - self.error_func()
            self.e_d = new_error - self.e
            self.e_i += new_error
            self.e = new_error

class RPM_Handler:
    
    rpm_interval = 0.2

    def __init__(self, motor):
        self.__target_motor = motor
        self.__timer = Timer()
        self.last_angle = self.__target_motor.get_degrees_counted() # I don't like the coupling but we're stuck in monolith dev anyway
        self.rpm = None

    def update_RPM(self): # change to yields so an interval can be forced
        self.__timer.reset()
        while self.__timer.now() < self.rpm_interval:
            yield self.rpm
        current_angle = self.get_degrees_counted() # process this to account for negatives. use total angle turned perhaps?
        self.rpm = (current_angle - self.last_angle) / self.__timer.now()
        self.last_angle = current_angle
        yield self.rpm

    
# need to add RPM calculation to accurately control motor speed. Position cannot be used as a stable speed is important to maintain pose
class PID_Motor(Motor):

    def __init__(self, port, forward = True, k = 0., t_i = 0., t_d = 0., backlash = 0) -> None:
        super().__init__(port)
        self.direction =  1 if forward else -1
        self.k = k
        self.t_i = t_i
        self.t_d = t_d
        self.backlash = backlash
        self.__error = Error(self.get_degrees_counted)
        self.__error.set_target(None)
        self.rpm_handler = RPM_Handler(self)
        self.rpm_generator = self.rpm_handler.update_RPM()
        self.rpm = next(self.rpm_generator)

    # MUST BE CALLED EVERY MAIN CYCLE AS WELL
    def update_rpm(self):
        self.rpm = next(self.rpm_generator) 

    def set_target(self, target):
        self.__error.set_target(target)

    def pid_calc(self, power):
        self.update_RPM() # just in case
        self.__error.update_error()
        pid_term = self.k * ( self.__error.e + 1. / self.t_i* self.__error.e_i + self.t_d * self.__error.e_d)
        return power #  + pid_term #done correctly this should not be two parts added together

    def start_at_power(self, power):
        raise Exception("defunct function")

    def start_at_rpm(self, power):
        self.set_target(power)
        new_power = self.pid_calc(power)
        super.start_at_power(new_power)

# things that would usually be in a config file
# lowest measured peak rpm was 151, we will target for 135rpm of that as our max output to allow for PID to adjust
# in practice we will aim for at most 50% of that value to avoid instability
wheel_base_cm = 12.2
wheel_diameter_cm = 5.6
l_ultrasonic_offset, r_ultrasonic_offset = 15.5/2
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
l_motor = Motor(l_motor)
r_motor = Motor(r_motor)
f_ultrasonic = DistanceSensor(f_ultrasonic)
l_ultrasonic = DistanceSensor(l_ultrasonic)
r_ultrasonic = DistanceSensor(r_ultrasonic)


def wheel_travel_distance(rotations):
    return rotations*pi*wheel_diameter_cm



# Python doesn't have built in constants, but we're going to agree to never
# change the following variables.
CONST_MAX_YAW = 45
CONST_MIN_YAW = - CONST_MAX_YAW
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
    searchAndApproach()
    raise SystemExit


main()
