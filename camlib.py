import cv2
import numpy as np
import os

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = 'hide'
import pygame, pygame.camera
pygame.init()

window_open = False
cam_open = False
cam_exp = 0
cam = None
pg_cam_open = False
pg_cam_exp = 0
pg_cam = None


def get_frame_pygame():
    global pg_cam
    global pg_cam_open
    global pg_cam_exp
    # Handle initialization
    if not pg_cam_open:
        print('Initializing pygame camera...')
        pygame.camera.init()
        pg_cam = pygame.camera.Camera('/dev/video0', (640, 480))
        pg_cam.start()
        print('Camera initialized with the following properties:')
        print('  - Size: {}'.format(pg_cam.get_size()))
        pg_cam_open = True
    frame = pygame.surfarray.array3d(pg_cam.get_image())
    # Fix rotation
    frame = frame.swapaxes(0, 1)
    # Fix color channels
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return np.zeros((640, 480, 3))
    return frame


def get_frame():
    global cam
    global cam_open
    global cam_exp
    # Handle initialization
    if not cam_open:
        print('Initializing camera...')
        cam = cv2.VideoCapture(0)
        print('Camera initialized with the following properties:')
        print('  - FPS:  {}'.format(round(cam.get(cv2.CAP_PROP_FPS))))
        print('  - Size: ({}, {})'.format(
            round(cam.get(cv2.CAP_PROP_FRAME_WIDTH)),
            round(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ))
        cam_open = True

    # Read frame
    status, frame = cam.read()
    if status:
        return frame
    return False


def show_frame(im):
    global window_open
    # Handle initialization
    if not window_open:
        print('Initializing image window...')
        cv2.namedWindow('Live Feed', cv2.WINDOW_NORMAL)
        window_open = True
    # Show frame
    cv2.imshow('Live Feed', im)
    cv2.waitKey(1)


def encode_jpg_bytes(im, quality=90, scale=1):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    if scale != 1:
        im = cv2.resize(im, (0, 0), fx=scale, fy=scale)
    jpg_im = cv2.imencode('.jpg', im, encode_param)[1]
    return jpg_im.tobytes()


def decode_jpg_bytes(jpg):
    jpg = np.frombuffer(jpg, dtype=np.uint8)
    im = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
    return im

