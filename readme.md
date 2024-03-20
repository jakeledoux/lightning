# lightning

Lightning McQueen, a (will be) self-driving tiny electric car. This repository
will contain all the code necessary for remotely controlling it with a gamepad,
live video streaming, and running the self-driving model. Model training will
likely be done outside this repo.

## Usage

Running the binary (`lightning` if installed, or `cargo run --release`) will
print out usage instructions.

`lightning controller` requires a supported bluetooth or USB gamepad to be
connected.

### Example

1. On a MacBook with a connected controller:
    - `lightning controller`
2. On a Raspberry Pi connected to the Arduino over USB:
    - `lightning car /dev/ttyACM0 [macbook address]:25565`
3. You should see the logs confirm a successful connection. After a second or
   two, you will be able to steer the car with the left stick of your gamepad.
   Forward and reverse is mapped to the right and left triggers respectively.
    - For safety, throttle is limited to 0 by default. Run with `--throttle-limit
      1.0` to enable max throttle.

## Resources

### Pairing bluetooth on Raspberry Pi

[RPi bluetoothctl example](https://bluedot.readthedocs.io/en/latest/pairpipi.html#using-the-command-line)

Once paired, issue the command `trust [address]` to automatically accept future
connections with that controller.
