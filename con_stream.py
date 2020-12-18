import camlib
import json
import re
from simpleeval import simple_eval
import socket
import sys
import time

# Regex patterns
kwarg_pattern = re.compile(r'-{0,2}(\w+)=(\S+)')


def get_command(idx):
    if len(sys.argv) > idx + 1:
        return sys.argv[idx + 1].lower().strip()
    else:
        return False


def get_kwarg(key, default):
    for arg in sys.argv[1:]:
        match = kwarg_pattern.match(arg)
        if match:
            arg_key, arg_value = match.groups()
            if arg_key == key:
                return simple_eval(arg_value)
    return default


def update_controls(controls):
    controls = json.loads(controls.decode('ascii'))
    print(controls)


# Global variables
HOST = '0.0.0.0'
PORT = 1984
DELIMITER = b'\n</con>'
POLL_RATE = get_kwarg('pollrate', 20)

role = get_command(0)
if role:
    # The server will recieve and view the image stream
    if role == 'server':
        print('Listening on {}:{}'.format(HOST, PORT))
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind((HOST, PORT))
            sock.settimeout(5)
            sock.listen()
            conn, addr = sock.accept()
            with conn:
                print('Connection established with {}:{}'.format(*addr))
                last_tx = time.time()
                while True:
                    # Enforce rate limit
                    if time.time() - last_tx > 1 / POLL_RATE:
                        try:
                            controls = {'steer': 10, 'gas': 100}
                            conn.sendall(json.dumps(controls).encode() + DELIMITER)
                            last_tx = time.time()
                        except (ConnectionResetError, BrokenPipeError):
                            print('Connection terminated')
                            break
    # The client will capture and transmit the image stream
    elif role == 'client':
        address = get_command(1)
        if address:
            print('Attempting connection to {}:{}'.format(address, PORT))
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((address, PORT))
                buffer = b''
                while True:
                    data = sock.recv(32)
                    if data:
                        buffer += data
                    else:
                        print('Connection terminated')
                        break
                    if DELIMITER in buffer:
                        *controls, buffer = buffer.split(DELIMITER)
                        update_controls(controls[-1])
        else:
            print('No server address specified')
    else:
        print('Invalid role')
else:
    print('Must specify role (server|client)')
