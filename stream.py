import camlib
import socket
import sys

HOST = '0.0.0.0'
PORT = 1984
DELIMITER = b'\n</img>'


def get_command(idx):
    if len(sys.argv) > idx + 1:
        return sys.argv[idx + 1].lower().strip()
    else:
        return False


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
                            img = camlib.decode_jpg_bytes(frame)
                            camlib.show_frame(img)
    # The client will capture and transmit the image stream
    elif role == 'client':
        address = get_command(1)
        if address:
            print('Attempting connection to {}:{}'.format(address, PORT))
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((address, PORT))
                while True:
                    try:
                        frame = camlib.get_frame()
                        frame = camlib.encode_jpg_bytes(frame)
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
