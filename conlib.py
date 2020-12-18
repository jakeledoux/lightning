import inputs
import threading

controls = {'steer': 0, 'gas': 0, 'beep': 0}
stop_threads = False

class ControlLoop:
    def __init__(self):
        self.event_handler = threading.Thread(target=update_controls)

    def __enter__(self):
        self.event_handler.start()
        return self

    def __exit__(self, type, value, traceback):
        global stop_threads
        stop_threads = True
        self.event_handler.join()

    @staticmethod
    def get_controls():
        global controls
        return controls

def update_controls():
    global controls

    while True:
        events = inputs.get_gamepad()
        for event in events:
            if event.code == 'ABS_X':
                controls['steer'] = round(event.state / 32768, 2)
            elif event.code == 'ABS_RZ':
                controls['gas'] = round(event.state / 255, 2)
            elif event.code == 'ABS_Z':
                controls['gas'] = round(-event.state / 255, 2)
            elif event.code == 'BTN_SOUTH':
                controls['beep'] = event.state
        global stop_threads
        if stop_threads:
            break


if __name__ == '__main__':
    with ControlLoop() as controller:
        while True:
            print(controller.get_controls())
