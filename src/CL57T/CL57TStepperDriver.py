#pylint: disable=invalid-name
#pylint: disable=no-member
#pylint: disable=too-many-arguments
#pylint: disable=too-many-public-methods
#pylint: disable=too-many-branches
#pylint: disable=too-many-instance-attributes
#pylint: disable=import-outside-toplevel
#pylint: disable=bare-except
"""CL57T stepper driver module

this module has two different functions:
1. move the motor via STEP/DIR pins
"""
import math
import time

from ._CL57T_GPIO_board import GPIO, BOARD
from ._CL57T_logger import CL57T_logger, Loglevel
from ._CL57T_move import MovementAbsRel, MovementPhase, StopMode




class CL57TStepperDriver:
    """CL57T

    this class has two different functions:
    1. move the motor via STEP/DIR pins
    """
    from ._CL57T_move import (
        set_movement_abs_rel, get_current_position, set_current_position, set_max_speed,
        set_max_speed_fullstep, get_max_speed, set_acceleration, set_acceleration_fullstep,
        get_acceleration, stop, get_movement_phase, run_to_position_steps,
        run_to_position_revolutions, run_to_position_steps_threaded,
        run_to_position_revolutions_threaded, wait_for_movement_finished_threaded, run,
        distance_to_go, compute_new_speed, run_speed, make_a_step, run_to_position_mm
    )

    BOARD = BOARD
    cl57t_logger = None
    _pin_step = -1
    _pin_dir = -1
    _pin_en = -1
    _pin_homing_sensor = -1

    _direction = True

    _stop = StopMode.NO
    _starttime = 0

    _msres = -1
    _steps_per_rev = 0
    _steps_per_mm = 0
    _fullsteps_per_rev = 1

    _homing_position_mm = 0                 # homing position in mm
    _current_pos = 0                 # current position of stepper in steps
    _target_pos = 0                  # the target position in steps
    _speed = 0.0                    # the current speed in steps per second
    _max_speed = 1.0                 # the maximum speed in steps per second
    _max_speed_homing = 200           # the maximum speed in steps per second for homing
    _acceleration = 1.0             # the acceleration in steps per second per second
    _acceleration_homing = 10000     # the acceleration in steps per second per second for homing
    _sqrt_twoa = 1.0                # Precomputed sqrt(2*_acceleration)
    _step_interval = 0               # the current interval between two steps
    _min_pulse_width = 1              # minimum allowed pulse with in microseconds
    _last_step_time = 0               # The last step time in microseconds
    _n = 0                          # step counter
    _c0 = 0                         # Initial step size in microseconds
    _cn = 0                         # Last step size in microseconds
    _cmin = 0                       # Min step size in microseconds based on maxSpeed
    _movement_phase = MovementPhase.STANDSTILL

    _movement_thread = None

    _deinit_finished = False



    def __init__(self, pin_en, pin_step=-1, pin_dir=-1,  pin_homing_sensor=-1, gearwheel_diameter_mm=-1, gpio_mode=GPIO.BCM, microstepping_resolution=1, loglevel=None, ):
        """constructor

        Args:
            pin_en (int): EN pin number
            pin_step (int, optional): STEP pin number. Defaults to -1.
            pin_dir (int, optional): DIR pin number. Defaults to -1.
            gpio_mode (enum, optional): gpio mode. Defaults to GPIO.BCM.
            loglevel (enum, optional): loglevel. Defaults to None.
        """
        self.cl57t_logger = CL57T_logger(loglevel, f"CL57T")


        self.cl57t_logger.log("Init", Loglevel.INFO)
        GPIO.setwarnings(False)
        GPIO.setmode(gpio_mode)

        self.cl57t_logger.log(f"EN Pin: {pin_en}", Loglevel.DEBUG)
        self._pin_en = pin_en
        GPIO.setup(self._pin_en, GPIO.OUT, initial=GPIO.HIGH)

        self.cl57t_logger.log(f"STEP Pin: {pin_step}", Loglevel.DEBUG)
        if pin_step != -1:
            self._pin_step = pin_step
            GPIO.setup(self._pin_step, GPIO.OUT, initial=GPIO.LOW)

        self.cl57t_logger.log(f"DIR Pin: {pin_dir}", Loglevel.DEBUG)
        if pin_dir != -1:
            self._pin_dir = pin_dir
            GPIO.setup(self._pin_dir, GPIO.OUT, initial=self._direction)

        self._msres = microstepping_resolution
        self._pin_homing_sensor = pin_homing_sensor
        distance_per_revolution = math.pi * gearwheel_diameter_mm
        self._steps_per_mm = self.read_steps_per_rev() / distance_per_revolution
        self.cl57t_logger.log(f"_steps_per_mm: {self._steps_per_mm}", Loglevel.DEBUG)

        self.cl57t_logger.log("GPIO Init finished", Loglevel.INFO)

        self.cl57t_logger.log("Init finished", Loglevel.INFO)

        self.set_max_speed_fullstep(100)
        self.set_acceleration_fullstep(100)



    def __del__(self):
        """destructor"""
        if self._deinit_finished is False:
            self.cl57t_logger.log("Deinit", Loglevel.INFO)

            self.set_motor_enabled(False)

            self.cl57t_logger.log("GPIO cleanup")
            if self._pin_step != -1:
                GPIO.cleanup(self._pin_step)
            if self._pin_dir != -1:
                GPIO.cleanup(self._pin_dir)
            if self._pin_en != -1:
                GPIO.cleanup(self._pin_en)
            if self._pin_homing_sensor != -1:
                GPIO.remove_event_detect(self._pin_homing_sensor)
                GPIO.cleanup(self._pin_homing_sensor)

            self.cl57t_logger.log("Deinit finished", Loglevel.INFO)
            self._deinit_finished= True
        else:
            self.cl57t_logger.log("Deinit already finished", Loglevel.INFO)
        del self.cl57t_logger



    def set_deinitialize_true(self):
        """set deinitialize to true"""
        self._deinit_finished = True



    def set_motor_enabled(self, en):
        """enables or disables the motor current output

        Args:
            en (bool): whether the motor current output should be enabled
        """
        GPIO.output(self._pin_en, not en)
        self.cl57t_logger.log(f"Motor output active: {en}", Loglevel.INFO)



    def do_homing(self):
        """homes the motor in the given direction using stallguard.
        this method is using vactual to move the motor and an interrupt on the DIAG pin

        Returns:
            not homing_failed (bool): true when homing was successful
        """
        self.cl57t_logger.log("---", Loglevel.INFO)
        self.cl57t_logger.log("homing", Loglevel.INFO)

        self.cl57t_logger.log(
            f"setup homing sensor callback on GPIO {self._pin_homing_sensor}",
            Loglevel.INFO,
        )

        def homing_sensor_value():
            return GPIO.input(self._pin_homing_sensor) == 0

        def homing_sensor_callback(channel):
            self._homing_sensor_touched = get_homing_sensor_state()
            self.cl57t_logger.log(f"_homing_sensor_touched: {self._homing_sensor_touched}", Loglevel.INFO)
            self._stop = StopMode.HARDSTOP
            self._current_pos = 0

        GPIO.setup(self._pin_homing_sensor, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        def do_not_touch_homing_sensor():
            self.cl57t_logger.log("moving back so homing sensor is not touched", Loglevel.INFO)
            if homing_sensor_value():
                self.run_to_position_steps(
                    999999999,
                    movement_abs_rel=MovementAbsRel.RELATIVE,
                    stop_condition=lambda : homing_sensor_value() is False
                )

                self.cl57t_logger.log("moving back a bit more", Loglevel.INFO)
                self.run_to_position_steps(50, movement_abs_rel=MovementAbsRel.RELATIVE)

        do_not_touch_homing_sensor()

        self.cl57t_logger.log("touching the homing sensor", Loglevel.INFO)
        self.run_to_position_steps(
            -999999999,
            movement_abs_rel=MovementAbsRel.RELATIVE,
            stop_condition=lambda : homing_sensor_value() is True
        )
        self.cl57t_logger.log("first homing reached", Loglevel.INFO)

        do_not_touch_homing_sensor()

        self.cl57t_logger.log("touching the homing sensor slower", Loglevel.INFO)
        self.set_acceleration(0)
        self.set_max_speed(50)
        self.run_to_position_steps(
            -999999999,
            movement_abs_rel=MovementAbsRel.RELATIVE,
            stop_condition=lambda : homing_sensor_value() is True
        )

        self.cl57t_logger.log("second homing reached", Loglevel.INFO)

        self.cl57t_logger.log("---", Loglevel.INFO)
        return True


    def reverse_direction_pin(self):
        """reverses the motor shaft direction"""
        self._direction = not self._direction
        GPIO.output(self._pin_dir, self._direction)



    def set_direction_pin(self, direction):
        """sets the motor shaft direction to the given value: 0 = CCW; 1 = CW

        Args:
            direction (bool): motor shaft direction: False = CCW; True = CW
        """
        self._direction = direction
        GPIO.output(self._pin_dir, direction)



    def read_steps_per_rev(self):
        """returns how many steps are needed for one revolution.
        this reads the value from the stepper driver.

        Returns:
            int: Steps per revolution
        """
        self._steps_per_rev = self._fullsteps_per_rev*self._msres
        return self._steps_per_rev



    def get_steps_per_rev(self):
        """returns how many steps are needed for one revolution.
        this gets the cached value from the library.

        Returns:
            int: Steps per revolution
        """
        return self._steps_per_rev


    def set_homing_position_mm(self, homing_position_mm):
        self._homing_position_mm = homing_position_mm

