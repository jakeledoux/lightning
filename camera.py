import socket
import sys

HOST = '127.0.0.1'
PORT = 1984


def get_command(idx):
    if len(sys.argv) > idx + 1:
        return sys.argv[idx + 1].lower().strip()
    else:
        return False


role = get_command(0)
if role:
    if role == 'server':
        print('Listening on {}:{}'.format(HOST, PORT))
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind((HOST, PORT))
            sock.listen()
            conn, addr = sock.accept()
            with conn:
                print('Connection established with {}:{}'.format(*addr))
                while True:
                    data = conn.recv(1024)
                    if not data:
                        print('Connection terminated')
                        break
                    conn.sendall(data)
    elif role == 'client':
        address = get_command(1)
        if address:
            print('Attempting connection to {}:{}'.format(address, PORT))
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((address, PORT))
                sock.sendall(b'Hello, world!')
                data = sock.recv(1024)
            print('Receieved', repr(data))
        else:
            print('No server address specified')
    else:
        print('Invalid role')
else:
    print('Must specify role (server|client)')
