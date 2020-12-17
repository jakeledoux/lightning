import socket
import sys

HOST = '0.0.0.0'
PORT = 1984
DELIMITER = b'\n'


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
                        print(frames)
                        for frame in frames:
                            print(frame)
    # The client will capture and transmit the image stream
    elif role == 'client':
        address = get_command(1)
        if address:
            print('Attempting connection to {}:{}'.format(address, PORT))
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((address, PORT))
                while True:
                    try:
                        sock.sendall(b'Did you ever hear the tragedy of Darth Plagueis The Wise? I thought not. It’s not a story the Jedi would tell you. It’s a Sith legend. Darth Plagueis was a Dark Lord of the Sith, so powerful and so wise he could use the Force to influence the midichlorians to create life… He had such a knowledge of the dark side that he could even keep the ones he cared about from dying. The dark side of the Force is a pathway to many abilities some consider to be unnatural. He became so powerful… the only thing he was afraid of was losing his power, which eventually, of course, he did. Unfortunately, he taught his apprentice everything he knew, then his apprentice killed him in his sleep. Ironic. He could save others from death, but not himself.' + DELIMITER)
                    except ConnectionResetError, BrokenPipeError:
                        print('Connection terminated')
            print('Receieved', repr(data))
        else:
            print('No server address specified')
    else:
        print('Invalid role')
else:
    print('Must specify role (server|client)')
