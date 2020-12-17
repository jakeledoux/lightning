import cv2
import numpy as np

window_open = False
cam_open = False
cam_exp = 0
cam = None


def get_frame():
    global cam
    global cam_open
    global cam_exp
    # Handle initialization
    if not cam_open:
        print('Initializing camera...')
        cam = cv2.VideoCapture(0)
        print('Camera initialized with the following properties:')
        print('  - FPS:  {}'.format(cam.get(cv2.CAP_PROP_FPS)))
        print('  - Size: {}'.format(cam.get(cv2.CAP_PROP_SIZE)))
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

