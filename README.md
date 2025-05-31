# Raspberry PI Stepper Driver for CL57T

[![PyPI python version TMC-2209-Raspberry-Pi](https://badgen.net/pypi/python/cl57t-raspberry-pi-stepper-drive)](https://pypi.org/project/cl57t-raspberry-pi-stepper-drive)
[![PyPI version TMC-2209-Raspberry-Pi](https://badgen.net/pypi/v/cl57t-raspberry-pi-stepper-drive)](https://pypi.org/project/cl57t-raspberry-pi-stepper-drive)
[![PyPI downloads TMC-2209-Raspberry-Pi](https://img.shields.io/pypi/dm/cl57t-raspberry-pi-stepper-drive)](https://pypi.org/project/cl57t-raspberry-pi-stepper-drive)
[![GitHub issues](https://img.shields.io/github/issues/iosifnicolae2/cl57t_raspberry_pi_stepper_driver.svg)](https://GitHub.com/iosifnicolae2/cl57t_raspberry_pi_stepper_driver/issues/)

\
\
This is a library to drive a stepper motor with a CL57T stepper driver and a Raspberry Pi.

This code is still experimental, so use it on your own risk.

This library is programmed in pure Python. The performance of Python is not good enough to drive the motor with high speed.
So if you move the motor with high speed and this library the motor will lose steps.

This library is a fork of [cl57t-raspberry-pi-stepper-drive](https://github.com/iosifnicolae2/cl57t-raspberry-pi-stepper-drive). Please check out this repo for more context.

the Documentation of the CL57T can be found here:  
[CL57T - Datsheet](https://www.omc-stepperonline.com/download/CL57T_V4.0.pdf)

The code is also available on [PyPI](https://pypi.org/project/cl57t-raspberry-pi-stepper-drive).

## Installation

### Installation with PIP

```shell
pip3 install cl57t-raspberry-pi-stepper-drive
```

## Wiring

Pin CL57T | connect to | Function
-- | -- | --
COM,ENA-, DIR-, PUL- | GND of Raspberry Pi | GND for VDD and Signals
ALM | GPIO 26 (pin 37) | receive an error signal from stepper driver
ENA | GPIO 19 (pin 35) | enable the motor output
DIR | GPIO 13 (pin 33) | set the direction of the motor
PUL/STEP | GPIO 6 (pin 31) | moves the motor one step per pulse
HOMING_SENSOR | GPIO 5 (pin 29) | optional, used for homing the stepper

The GPIO pins can be specific when initiating the class.

## Sample Code
```python
#pylint: disable=wildcard-import
#pylint: disable=unused-wildcard-import
#pylint: disable=unused-import
#pylint: disable=duplicate-code
"""
test file for testing basic movement
"""
from time import sleep
from cl57t_raspberry_pi_stepper_drive.CL57TStepperDriver import *

print("---")
print("SCRIPT START")
print("---")

# Pinout
# GPIO 26 (pin 37): ALM
# GPIO 19 (pin 35): ENA
# GPIO 13 (pin 33): DIR
# GPIO 6 (pin 31): PUL/STEP
# GPIO 5 (pin 29): HOMING_SENSOR

#-----------------------------------------------------------------------
# initiate the CL57T class
# use your pins for pin_en, pin_step, pin_dir here
#-----------------------------------------------------------------------
stepper = CL57TStepperDriver(
    pin_en=19, # GPIO 19 (pin 35): ENA
    pin_step=6, # GPIO 6 (pin 31): PUL/STEP
    pin_dir=13, # GPIO 13 (pin 33): DIR
    pin_homing_sensor=5, # GPIO 2 (pin 29): HOMING_SENSOR
    microstepping_resolution=1600,
    gearwheel_diameter_mm=56, # HTD 36 5M 09
    loglevel=Loglevel.DEBUG,
)

#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
stepper.cl57t_logger.set_loglevel(Loglevel.DEBUG)
stepper.set_movement_abs_rel(MovementAbsRel.ABSOLUTE)

print("---\n---")


#-----------------------------------------------------------------------
# activate the motor current output
#-----------------------------------------------------------------------
stepper.set_motor_enabled(True)

stepper.set_acceleration(800)
stepper.set_max_speed(800)
stepper.do_homing()

stepper.set_acceleration(1600 * 10)
stepper.set_max_speed(10000)


stepper.set_homing_position_mm(400)

stepper.run_to_position_mm(1000)

stepper.run_to_position_mm(400)

immediately. Call :py:meth:`stop` from another thread to abort the movement or
use ``wait_for_movement_finished_threaded()`` to wait for it to finish.


#-----------------------------------------------------------------------
# move the motor 1 revolution
#-----------------------------------------------------------------------
# stepper.run_to_position_steps(9)                             #move to position 400
# stepper.run_to_position_steps(0)                               #move to position 0
#
#
# stepper.run_to_position_steps(400, MovementAbsRel.RELATIVE)    #move 400 steps forward
# stepper.run_to_position_steps(-400, MovementAbsRel.RELATIVE)   #move 400 steps backward
#
#
# stepper.run_to_position_steps(400)                             #move to position 400
# stepper.run_to_position_steps(0)                               #move to position 0

#

#-----------------------------------------------------------------------
# move the motor 1000 steps or 2 seconds (which ever is lower)
#-----------------------------------------------------------------------
#stepper.run_to_position_steps(1000, blocking=False)
#sleep(2)
#stepper.stop()


#-----------------------------------------------------------------------
# deactivate the motor current output
#-----------------------------------------------------------------------
stepper.set_motor_enabled(False)

print("---\n---")


#-----------------------------------------------------------------------
# deinitiate the CL57T class
#-----------------------------------------------------------------------
del stepper

print("---")
print("SCRIPT FINISHED")
print("---")


```

## Acknowledgements
This library is a fork of [TMC2209_Raspberry_Pi
](https://github.com/Chr157i4n/TMC2209_Raspberry_Pi).

The code to run the stepper motor is based on the code of the [AccelStepper Library from Mike McCauley](http://www.airspayce.com/mikem/arduino/AccelStepper).

The main focus for this are Test setups, as Python is not fast enough for high motor speeds.

## Feedback/Contributing

If you encounter any problem, feel free to open an issue on the Github [issue page](https://github.com/iosifnicolae2/cl57t-raspberry-pi-stepper-drive/issues).
Feedback will keep this project growing and I encourage all suggestions.
Feel free to submit a pull request on the dev branch.
