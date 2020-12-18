""" This module is responsible for managing communication between the Raspberry
    Pi and car hardware.
"""

from gpiozero import AngularServo
import time

# Constants (recommended that these are set programmatically rather than by
# editing this source file.
STEER_PIN = 18
STEER_GAIN = 75  # Max 'angle' of steering servos. Angle is not 1:1 with real
                 # life and tends to max out around 80-85

# Servos
steer_servo = None

def init(do_wiggle=False):
    global steer_servo
    steer_servo = AngularServo(STEER_PIN)

    if do_wiggle:
        steer_servo.min()
        time.sleep(0.5)
        steer_servo.max()
        time.sleep(0.5)
        steer_servo.mid()


def update_from_dict(controls: dict):
    global STEER_GAIN
    global steer_servo

    if steer_servo:
        steer_servo.angle = controls['steer'] * STEER_GAIN
    else:
        print('Steering servo not initialized. Did you call carlib.init()?')


def steer(value: float):
    ''' Update the steering without the need for a dictionary.
    '''
    update_from_dict({'steer': value})


def gas(value: float):
    ''' Update the throttle without the need for a dictionary.
    '''
    update_from_dict({'gas': value})


def panic():
    ''' Neutralizes all controls.
    '''
    global steer_servo

    if steer_servo:
        steer_servo.mid()


def close():
    ''' Reset controls and close connections.
    '''
    global steer_servo

    panic()
    time.sleep(0.5)
    if steer_servo:
        steer_servo.close()
