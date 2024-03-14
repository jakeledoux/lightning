import json
import socket
import time

from lib.controller import Controller, ControlFrame
from lib.iolib import *


def update_controls(controls):
    global PRINT_CONTROLS
    controls = ControlFrame.from_json(controls.decode('ascii'))
    if PRINT_CONTROLS:
        print(controls)
    carlib.update_from_dict(controls.to_dict())


# Global variables
HOST = '0.0.0.0'
PORT = get_kwarg('port', 1988)
DELIMITER = b'\n</con>'
POLL_RATE = get_kwarg('pollrate', 70)
PRINT_CONTROLS = get_kwarg('print', False)
TIMEOUT = get_kwarg('timeout', 5)
SERIAL = get_kwarg('serial', None)

role = get_command(0)
if role:
    # The server will recieve and view the image stream
    if role == 'server':
        controller = Controller()
        print('Listening on {}:{}'.format(HOST, PORT))
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind((HOST, PORT))
            sock.settimeout(TIMEOUT)
            sock.listen()
            conn, addr = sock.accept()
            with conn:
                print('Connection established with {}:{}'.format(*addr))
                last_tx = time.time()
                while True:
                    # Enforce rate limit
                    if time.time() - last_tx > 1 / POLL_RATE:
                        try:
                            controls = controller.poll()
                            if controls:
                                conn.sendall(controls.to_json().encode() + DELIMITER)
                                last_tx = time.time()
                            else:
                                print('Gamepad not connected.')
                                break
                        except (ConnectionResetError, BrokenPipeError):
                            print('Connection terminated')
                            break
    # The client will capture and transmit the image stream
    elif role == 'client':
        address = get_command(1)
        if address:
            print('Initializing car')
            from lib import carlib
            print(SERIAL)
            if carlib.init(SERIAL):
                print('Attempting connection to {}:{}'.format(address, PORT))
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.settimeout(TIMEOUT)
                    sock.connect((address, PORT))
                    buffer = b''
                    while True:
                        try:
                            data = sock.recv(32)
                            if data:
                                buffer += data
                            else:
                                break
                            if DELIMITER in buffer:
                                *controls, buffer = buffer.split(DELIMITER)
                                update_controls(controls[-1])
                        except ConnectionResetError:
                            break
                print('Connection terminated')
                carlib.close()
        else:
            print('No server address specified')
    else:
        print('Invalid role')
else:
    print('Must specify role (server|client)')
