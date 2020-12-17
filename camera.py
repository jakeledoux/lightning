import socket
import sys

HOST = '0.0.0.0'
PORT = 1984


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
                print('Connection established with {}:{}'.format(*addr))
                while True:
                    data = conn.recv(1024)
                    if not data:
                        print('Connection terminated')
                        break
                    print(data)
    # The client will capture and transmit the image stream
    elif role == 'client':
        address = get_command(1)
        if address:
            print('Attempting connection to {}:{}'.format(address, PORT))
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((address, PORT))
                while True:
                    sock.sendall(b'Hello, world!')
            print('Receieved', repr(data))
        else:
            print('No server address specified')
    else:
        print('Invalid role')
else:
    print('Must specify role (server|client)')
