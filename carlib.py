""" This module is responsible for managing communication between the Raspberry
    Pi and car hardware.
"""
import serial
import time

# Constants (recommended that these are set programmatically rather than by
# editing this source file.
STEER_GAIN = 40
STEER_MID = 90
THROTTLE_GAIN = 100
SERIAL_PIN = '/dev/ttyACM0'
DELAY = 0.005

car_serial = None
last_write = time.time()

def init():
    global car_serial
    global SERIAL_PIN
    car_serial = serial.Serial(SERIAL_PIN, 9600)


def update_from_dict(controls: dict):
    global car_serial
    global DELAY
    global last_write
    global STEER_GAIN
    global STEER_MID
    global THROTTLE_GAIN

    if time.time() - last_write > DELAY:
        if car_serial:
            steer_angle = int(controls.get('steer', 0) * STEER_GAIN + STEER_MID)
            throttle = int(controls.get('gas', 0) * THROTTLE_GAIN)
            car_serial.write('s{}t{}'.format(steer_angle, throttle).encode())
            last_write = time.time()
        else:
            print('Serial communication not initialized. Did you call carlib.init()?')


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
