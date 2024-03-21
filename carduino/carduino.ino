#include <Servo.h>

#define BAUD_RATE 38400
#define HANDSHAKE 0xFF

#define STEER_PIN 5
#define MOTOR_PIN 3
#define BUZZER_PIN 11

#define STEER_ENABLED true
#define MOTOR_ENABLED true
#define HORN_ENABLED true

#define PANIC_INTERVAL 5000

Servo motor_servo;
Servo steer_servo;

struct ControlMessage
{
    // neutral: 90, range: appx 45 to 135 (I'm guessing)
    uint8_t steering;
    // neutral: 0, range: appx -100 to 100
    int8_t throttle;
    // neutral: false
    bool horn;
};

const ControlMessage DEFAULT_CONTROL_MESSAGE = {90, 0, false};

int iterSinceLastCommand = 0;
ControlMessage msg = DEFAULT_CONTROL_MESSAGE;

void setup()
{
    // attach servos
    if (STEER_ENABLED)
    {
        steer_servo.attach(STEER_PIN);
    }
    if (MOTOR_ENABLED)
    {
        delay(1000);
        motor_servo.attach(MOTOR_PIN);
        motor_servo.writeMicroseconds(1500);
    }

    // establish serial connection
    Serial.begin(BAUD_RATE);
    Serial.setTimeout(3);
}

void loop()
{
    // update controls based on serial input
    if (Serial.available())
    {
        // receive incoming bytes
        char bytes[3];
        Serial.readBytes(bytes, 3);

        // deconstruct into message
        msg.steering = bytes[0];
        msg.throttle = (int8_t)bytes[1];
        msg.horn = (bool)bytes[2];

        // transmit checksum
        Serial.write(msg.steering + msg.throttle + msg.horn);
    }
    else
    {
        Serial.write((uint8_t)HANDSHAKE);
    }

    // reset controls if "timeout" reached
    if (iterSinceLastCommand++ > PANIC_INTERVAL)
    {
        msg = DEFAULT_CONTROL_MESSAGE;
        iterSinceLastCommand = 0;
    }

    // execute control commands
    if (STEER_ENABLED)
        steer_servo.write(msg.steering);
    if (MOTOR_ENABLED)
        motor_servo.writeMicroseconds(map(msg.throttle, 0, 100, 1500, 1800));
    if (HORN_ENABLED && msg.horn)
    {
        tone(BUZZER_PIN, 440, 30);
        msg.horn = false;
    }
}
