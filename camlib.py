import cv2
import numpy as np

window_open = False
cam_open = False
cam = None


def get_frame():
    global cam
    global cam_open
    # Handle initialization
    if not cam_open:
        cam = cv2.VideoCapture(0)
        cam.set(cv2.CV_CAP_PROP_AUTO_EXPOSURE, 1)

    # Read frame
    status, frame = cam.read()
    if status:
        return frame
    return False


def show_frame(im):
    global window_open
    # Handle initialization
    if not window_open:
        cv2.namedWindow('Live Feed', cv2.WINDOW_NORMAL)
        window_open = True
    # Show frame
    cv2.imshow('Live Feed', im)
    cv2.waitKey(1)


def encode_jpg_bytes(im, quality=90):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    jpg_im = cv2.imencode('.jpg', im, encode_param)[1]
    return jpg_im.tobytes()


def decode_jpg_bytes(jpg):
    jpg = np.frombuffer(jpg, dtype=np.uint8)
    im = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
    return im

