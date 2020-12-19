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

def init(serial_location=None):
    global car_serial
    global SERIAL_PIN
    try:
        car_serial = serial.Serial(serial_location or SERIAL_PIN, 9600,
                                   write_timeout=DELAY)
        return True
    except serial.serialutil.SerialException:
        print('Failed to connect to Arduino. Is it plugged in?')
        return False


def update_from_dict(controls: dict):
    global car_serial
    global DELAY
    global last_write
    global STEER_GAIN
    global STEER_MID
    global THROTTLE_GAIN

    if time.time() - last_write > DELAY:
        if car_serial and car_serial.writable():
            steer_angle = int(controls.get('steer', 0) * STEER_GAIN + STEER_MID)
            throttle = int(controls.get('gas', 0) * THROTTLE_GAIN)
            try:
                car_serial.write('<s{}t{}/>'.format(steer_angle, throttle).encode())
                last_write = time.time()
            except:
                print('Write timed-out.')
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
    update_from_dict({'steer': 0, 'gas': 0})


def close():
    ''' Reset controls and close connections.
    '''
    global car_serial

    panic()
    time.sleep(0.5)
    if car_serial:
        car_serial.close()
