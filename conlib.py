import inputs
import threading

controls = {'steer': 0, 'gas': 0, 'beep': 0}
stop_threads = False

class ControlLoop:
    def __init__(self):
        for device in inputs.devices:
            if device.name == 'Microsoft X-Box 360 pad':
                gamepad = device
                break
        else:
            raise Exception('Cannot find Xbox controller. Is it connected?')
        self.event_handler = threading.Thread(target=update_controls, args=[gamepad])
        self.event_handler.daemon = True

    def __enter__(self):
        self.event_handler.start()
        return self

    def __exit__(self, type, value, traceback):
        pass

    @staticmethod
    def get_controls():
        global controls
        return controls

def update_controls(gamepad):
    global controls

    gamepad.read_size = 1

    while True:
        for event in gamepad.read():
            if event.code == 'ABS_X':
                controls['steer'] = event.state / 32768
            elif event.code == 'ABS_RZ':
                controls['gas'] = event.state / 255
            elif event.code == 'ABS_Z':
                controls['gas'] = -event.state / 255
            elif event.code == 'BTN_SOUTH':
                controls['beep'] = event.state
            break


if __name__ == '__main__':
    import time
    poll_rate = 20
    with ControlLoop() as controller:
        last_poll = time.time()
        while True:
            if time.time() - last_poll >= 1 / poll_rate:
                print(controller.get_controls())
                last_poll = time.time()
