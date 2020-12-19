import re
from lib import camlib
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


# Global variables
HOST = '0.0.0.0'
PORT = 1984
DELIMITER = b'\n</img>'
JPG_QUALITY = get_kwarg('quality', 50)
IMG_SCALE = get_kwarg('scale', 1)

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
                frame_times = list()
                buffer = b''
                print('Connection established with {}:{}'.format(*addr))
                while True:
                    data = conn.recv(32)
                    if data:
                        buffer += data
                    else:
                        print('Connection terminated')
                        break
                    if DELIMITER in buffer:
                        *frames, buffer = buffer.split(DELIMITER)
                        for frame in frames:
                            # Log frame processed timestamps
                            frame_times.append(time.time())
                            img = camlib.decode_jpg_bytes(frame)
                            camlib.show_frame(img)
                            # Calculate framerate
                            if len(frame_times) >= 60:
                                duration = frame_times[-1] - frame_times[0]
                                print('Current framerate: {}'.format(
                                    round(1 / (duration / len(frame_times)))
                                ))
                                frame_times = list()
    # The client will capture and transmit the image stream
    elif role == 'client':
        address = get_command(1)
        if address:
            print('Attempting connection to {}:{}'.format(address, PORT))
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((address, PORT))
                while True:
                    try:
                        frame = camlib.get_frame_pygame()
                        frame = camlib.encode_jpg_bytes(frame, JPG_QUALITY,
                                                        scale=IMG_SCALE)
                        sock.sendall(frame + DELIMITER)
                    except (ConnectionResetError, BrokenPipeError):
                        print('Connection terminated')
                        break
        else:
            print('No server address specified')
    else:
        print('Invalid role')
else:
    print('Must specify role (server|client)')
