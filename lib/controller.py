# Thanks to https://blog.thea.codes/talking-to-gamepads-without-pygame/

from dataclasses import dataclass
from typing import Optional

import hid
from dataclasses_json import dataclass_json


@dataclass_json
@dataclass
class ControlFrame:
    steer: float
    gas: float
    beep: bool
    start: bool


DEFAULT_FRAME = ControlFrame(
    steer=0,
    gas=0,
    beep=False,
    start=False,
)


class Controller:
    def __init__(self, throttle_limit: Optional[float] = None):
        query_string = "dualsense"

        self.hid_gamepad = hid.device()
        self.last_report = None
        self.throttle_limit = throttle_limit

        # open device
        for device in hid.enumerate():
            if query_string.lower() in device['product_string'].lower():
                self.hid_gamepad.open(device['vendor_id'], device['product_id'])
                self.hid_gamepad.set_nonblocking(True)
                break
        else:
            raise ValueError("could not find matching device")

    def poll(self) -> ControlFrame:
        # attempt to update report
        report = self.hid_gamepad.read(273)
        if report:
            self.last_report = report

        # no report has yet been received
        if not self.last_report:
            return DEFAULT_FRAME

        report = self.last_report

        # the following values are configured for the dualsense controller over bluetooth
        # print(self.last_report)

        face_buttons = report[5]
        additional_buttons = report[6]

        return ControlFrame(
            steer=(report[1] / 128) - 1,
            gas=report[9] / 256 - report[8] / 256,
            beep=bool(0b00100000 & face_buttons),
            start=bool(0b00100000 & additional_buttons),
        )

    @staticmethod
    def list_devices():
        for device in hid.enumerate():
            print(device['product_string'])


if __name__ == '__main__':
    print("Entering debug mode. Press <start> to exit.")
    controller = Controller()
    while True:
        frame = controller.poll()
        print(frame)
        if frame.start:
            break
