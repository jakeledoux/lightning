from .xinput import get_bit_values, XInputJoystick

THROTTLE_LIMIT = 0.4

gamepad = XInputJoystick(0)
limit_throttle = True

def get_controls():
    global THROTTLE_LIMIT
    global gamepad
    global limit_throttle

    if gamepad is not None:
        try:
            state = gamepad.get_state().gamepad
        except AttributeError:
            return False

        buttons = get_bit_values(state.buttons)

        beep = buttons[17]

        # Engage throttle limiter if LB is pushed
        if buttons[23] and not limit_throttle:
            limit_throttle = True
            beep = True
            print('Engaged throttle limiter.')
        # Disengage throttle limiter if RB is pushed
        if buttons[22] and limit_throttle:
            limit_throttle = False
            beep = True
            print('Disengaged throttle limiter.')

        return {'steer': state.l_thumb_x / 32768,
                'gas': \
                    (state.right_trigger / 255 - state.left_trigger / 255) \
                    * (THROTTLE_LIMIT if limit_throttle else 1),
                'beep': beep,
                'start': buttons[27]}
    return False

if __name__ == '__main__':
    while True:
        controls = get_controls()
        print(controls)
        if controls['start']:
            break
