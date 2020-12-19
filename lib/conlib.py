from .xinput import get_bit_values, XInputJoystick

gamepad = XInputJoystick(0)

def get_controls():
    global gamepad
    state = gamepad.get_state().gamepad
    buttons = get_bit_values(state.buttons)

    return {'steer': state.l_thumb_x / 32768,
            'gas': (state.right_trigger / 255) - (state.left_trigger / 255),
            'beep': buttons[17],
            'start': buttons[27]}

if __name__ == '__main__':
    while True:
        controls = get_controls()
        print(controls)
        if controls['start']:
            break
