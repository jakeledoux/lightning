import inputs

controls = {'steer': 0, 'gas': 0, 'beep': 0}

def xbox():
    global controls

    while True:
        events = inputs.get_gamepad()
        for event in events:
            if event.code == 'ABS_X':
                controls['steer'] = event.state / 32768
            elif event.code == 'ABS_RZ':
                controls['gas'] = event.state / 255
            elif event.code == 'ABS_Z':
                controls['gas'] = -event.state / 255
            elif event.code == 'BTN_SOUTH':
                controls['beep'] = event.state

if __name__ == "__main__":
    xbox()
