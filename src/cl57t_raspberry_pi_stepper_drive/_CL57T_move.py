#pylint: disable=invalid-name
#pylint: disable=too-many-public-methods
#pylint: disable=too-many-branches
#pylint: disable=no-member
#pylint: disable=protected-access
#pylint: disable=bare-except
"""
CL57T stepper driver communication module
"""

import time
from enum import Enum
import math
import threading
from ._CL57T_GPIO_board import GPIO
from ._CL57T_logger import Loglevel



class Direction(Enum):
    """movement direction of the motor"""
    CCW = 0
    CW = 1


class MovementAbsRel(Enum):
    """movement absolute or relative"""
    ABSOLUTE = 0
    RELATIVE = 1


class MovementPhase(Enum):
    """movement phase"""
    STANDSTILL = 0
    ACCELERATING = 1
    MAXSPEED = 2
    DECELERATING = 3


class StopMode(Enum):
    """stopmode"""
    NO = 0
    SOFTSTOP = 1
    HARDSTOP = 2


def set_movement_abs_rel(self, movement_abs_rel):
    """set whether the movment should be relative or absolute by default.
    See the Enum MovementAbsoluteRelative

    Args:
        movement_abs_rel (enum): whether the movment should be relative or absolute
    """
    self._movement_abs_rel = movement_abs_rel



def get_current_position(self):
    """returns the current motor position in µsteps

    Returns:
        bool: current motor position
    """
    return self._current_pos



def set_current_position(self, new_pos):
    """overwrites the current motor position in µsteps

    Args:
        new_pos (bool): new position of the motor in µsteps
    """
    self._current_pos = new_pos



def set_max_speed(self, speed):
    """sets the maximum motor speed in µsteps per second

    Args:
        speed (int): speed in µsteps per second
    """
    if speed < 0.0:
        speed = -speed
    if self._max_speed != speed:
        self._max_speed = speed
        self._cmin = 1000000.0 / speed
        # Recompute _n from current speed and adjust speed if accelerating or cruising
        if self._n > 0:
            self._n = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
            self.compute_new_speed()



def set_max_speed_fullstep(self, speed):
    """sets the maximum motor speed in fullsteps per second

    Args:
        speed (int): maximum speed in fullsteps/sec
    """
    self.set_max_speed(speed*self._msres)



def get_max_speed(self):
    """returns the maximum motor speed in steps per second

    Returns:
        int: current maximum speed in steps/sec
    """
    return self._max_speed



def set_acceleration(self, acceleration):
    """sets the motor acceleration/decceleration in µsteps per sec per sec

    Args:
        acceleration (int): acceleration/decceleration in µsteps per sec per sec
    """
    if acceleration == 0.0:
        return
    acceleration = abs(acceleration)
    if self._acceleration != acceleration:
        # Recompute _n per Equation 17
        self._n = self._n * (self._acceleration / acceleration)
        # New c0 per Equation 7, with correction per Equation 15
        self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0 # Equation 15
        self._acceleration = acceleration
        self.compute_new_speed()



def set_acceleration_fullstep(self, acceleration):
    """sets the motor acceleration/decceleration in fullsteps per sec per sec

    Args:
        acceleration (int): acceleration/decceleration in fullsteps per sec per sec
    """
    self.set_acceleration(acceleration*self._msres)



def get_acceleration(self):
    """returns the motor acceleration/decceleration in steps per sec per sec

    Returns:
        int: acceleration/decceleration in µsteps per sec per sec
    """
    return self._acceleration



def stop(self, stop_mode = StopMode.HARDSTOP):
    """stop the current movement

    Args:
        stop_mode (enum): whether the movement should be stopped immediatly or softly
            (Default value = StopMode.HARDSTOP)
    """
    self._stop = stop_mode



def get_movement_phase(self):
    """return the current Movement Phase

    Returns:
        movement_phase (enum): current Movement Phase
    """
    return self._movement_phase


def _run_to_position_steps_blocking(self, steps, movement_abs_rel = MovementAbsRel.ABSOLUTE, stop_condition = None):
    """Internal helper that performs a blocking move."""
    if movement_abs_rel == MovementAbsRel.RELATIVE:
        self._target_pos = self._current_pos + steps
    else:
        self._target_pos = steps

    self._stop = StopMode.NO
    self._step_interval = 0
    self._speed = 0.0
    self._n = 0
    self.compute_new_speed()
    while self.run():
        if self._stop == StopMode.HARDSTOP or (stop_condition is not None and stop_condition()):
            break

    self._movement_phase = MovementPhase.STANDSTILL
    return self._stop


def run_to_position_steps(self, steps, movement_abs_rel = MovementAbsRel.ABSOLUTE, blocking = True):
    """Run the motor to the given position.
    With acceleration and deceleration. By default the call is **blocking** and
    the method returns only once the movement has finished or :func:`stop` has
    been called.  Passing ``blocking=False`` will start the movement in a
    background thread and return immediately.  The movement can then be aborted
    from another thread by calling :func:`stop`.

    Args:
        steps (int): amount of steps; can be negative.
        movement_abs_rel (MovementAbsRel): whether the movement should be
            absolute or relative.
        blocking (bool): when ``True`` the function blocks until finished.

    Returns:
        threading.Thread | StopMode: if ``blocking`` is ``False`` the created
        thread handling the movement is returned.  When ``blocking`` is
        ``True`` the :class:`StopMode` with which the movement finished is
        returned.
    """

    if blocking:
        return _run_to_position_steps_blocking(self, steps, movement_abs_rel, None)

    if self._movement_thread and self._movement_thread.is_alive():
        raise RuntimeError("Movement already running")

    self._movement_thread = threading.Thread(
        target=_run_to_position_steps_blocking,
        args=(self, steps, movement_abs_rel, None),
        daemon=True,
    )
    self._movement_thread.start()
    return self._movement_thread


def run_to_position_mm(self, mm, movement_abs_rel = MovementAbsRel.ABSOLUTE, blocking = True):
    """Run the motor to the given position in ``mm``.
    With acceleration and deceleration. Works the same way as
    :func:`run_to_position_steps` regarding the ``blocking`` behaviour.

    Args:
        mm (int): amount of millimetres; can be negative
        movement_abs_rel (enum): whether the movement should be absolute or relative
            (Default value = MovementAbsRel.ABSOLUTE)
        blocking (bool): when ``True`` the call blocks until finished.

    Returns:
        stop (enum) | threading.Thread: result of :func:`run_to_position_steps`
    """
    if movement_abs_rel == MovementAbsRel.RELATIVE:
        target_position_steps = self._current_pos + (mm * self._steps_per_mm)
    else:
        target_position_steps = (mm - self._homing_position_mm) * self._steps_per_mm

    target_position_steps = round(target_position_steps)
    print(f"target_position_steps: {target_position_steps}")
    return self.run_to_position_steps(
        target_position_steps,
        movement_abs_rel=MovementAbsRel.ABSOLUTE,
        blocking=blocking,
    )


def run_to_position_revolutions(self, revolutions, movement_abs_rel = MovementAbsRel.ABSOLUTE, blocking = True):
    """Run the motor to the given position specified in ``revolutions``.
    With acceleration and deceleration. Behaves exactly like
    :func:`run_to_position_steps` regarding the ``blocking`` flag.

    Args:
        revolutions (int): amount of revolutions; can be negative
        movement_abs_rel (enum): whether the movement should be absolute or relative
            (Default value = MovementAbsRel.ABSOLUTE)
        blocking (bool): when ``True`` the call blocks until finished.

    Returns:
        stop (enum) | threading.Thread: result of :func:`run_to_position_steps`
    """
    return self.run_to_position_steps(
        round(revolutions * self._steps_per_rev),
        movement_abs_rel,
        blocking=blocking,
    )



def run(self):
    """calculates a new speed if a speed was made

    returns true if the target position is reached
    should not be called from outside!
    """
    if self.run_speed(): #returns true, when a step is made
        self.compute_new_speed()
    return self._speed != 0.0 and self.distance_to_go() != 0



def distance_to_go(self):
    """returns the remaining distance the motor should run"""
    return self._target_pos - self._current_pos



def compute_new_speed(self):
    """returns the calculated current speed depending on the acceleration

    this code is based on:
    "Generate stepper-motor speed profiles in real time" by David Austin
    https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
    https://web.archive.org/web/20140705143928/http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf
    """
    distance_to = self.distance_to_go() # +ve is clockwise from curent location
    steps_to_stop = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
    if ((distance_to == 0 and steps_to_stop <= 2) or
    (self._stop == StopMode.SOFTSTOP and steps_to_stop <= 1)):
        # We are at the target and its time to stop
        self._step_interval = 0
        self._speed = 0.0
        self._n = 0
        self._movement_phase = MovementPhase.STANDSTILL
        self.cl57t_logger.log("time to stop", Loglevel.MOVEMENT)
        return

    if distance_to > 0:
        # We are anticlockwise from the target
        # Need to go clockwise from here, maybe decelerate now
        if self._n > 0:
            # Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((steps_to_stop >= distance_to) or self._direction == Direction.CCW or
                self._stop == StopMode.SOFTSTOP):
                self._n = -steps_to_stop # Start deceleration
                self._movement_phase = MovementPhase.DECELERATING
        elif self._n < 0:
            # Currently decelerating, need to accel again?
            if (steps_to_stop < distance_to) and self._direction == Direction.CW:
                self._n = -self._n # Start accceleration
                self._movement_phase = MovementPhase.ACCELERATING
    elif distance_to < 0:
        # We are clockwise from the target
        # Need to go anticlockwise from here, maybe decelerate
        if self._n > 0:
            # Currently accelerating, need to decel now? Or maybe going the wrong way?
            if (((steps_to_stop >= -distance_to) or self._direction == Direction.CW or
                self._stop == StopMode.SOFTSTOP)):
                self._n = -steps_to_stop # Start deceleration
                self._movement_phase = MovementPhase.DECELERATING
        elif self._n < 0:
            # Currently decelerating, need to accel again?
            if (steps_to_stop < -distance_to) and self._direction == Direction.CCW:
                self._n = -self._n # Start accceleration
                self._movement_phase = MovementPhase.ACCELERATING
    # Need to accelerate or decelerate
    if self._n == 0:
        # First step from stopped
        self._cn = self._c0
        GPIO.output(self._pin_step, GPIO.LOW)
        if distance_to > 0:
            self.set_direction_pin(1)
            self.cl57t_logger.log("going CW", Loglevel.MOVEMENT)
        else:
            self.set_direction_pin(0)
            self.cl57t_logger.log("going CCW", Loglevel.MOVEMENT)
        self._movement_phase = MovementPhase.ACCELERATING
    else:
        # Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
        self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1)) # Equation 13
        self._cn = max(self._cn, self._cmin)
        if self._cn == self._cmin:
            self._movement_phase = MovementPhase.MAXSPEED
    self._n += 1
    self._step_interval = self._cn
    self._speed = 1000000.0 / self._cn
    if self._direction == 0:
        self._speed = -self._speed



def run_speed(self):
    """this methods does the actual steps with the current speed"""
    # Dont do anything unless we actually have a step interval
    if not self._step_interval:
        return False

    curtime = time.time_ns()/1000

    if curtime - self._last_step_time >= self._step_interval:

        if self._direction == 1: # Clockwise
            self._current_pos += 1
        else: # Anticlockwise
            self._current_pos -= 1
        self.make_a_step()

        self._last_step_time = curtime # Caution: does not account for costs in step()
        return True
    return False



def make_a_step(self):
    """method that makes on step

    for the CL57T there needs to be a signal duration of minimum 100 ns
    """
    GPIO.output(self._pin_step, GPIO.HIGH)
    time.sleep(1/1000/1000)
    GPIO.output(self._pin_step, GPIO.LOW)
    time.sleep(1/1000/1000)

    self.cl57t_logger.log("one step", Loglevel.MOVEMENT)
