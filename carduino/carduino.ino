#include <Servo.h>

#define MOTOR_PIN 3
#define STEER_PIN 5
#define MOTOR_ENABLED true
#define STEER_ENABLED true
#define READBACK_ENABLED false

Servo motor_servo;
Servo steer_servo;
int steering = 90;
int throttle = 0;

void setup() {
  if (MOTOR_ENABLED) {
    delay(1000);
    motor_servo.attach(MOTOR_PIN);
    motor_servo.writeMicroseconds(1500);
  }
  if (STEER_ENABLED) {
    steer_servo.attach(STEER_PIN);
  }
  Serial.begin(9600);
  Serial.setTimeout(3);
  if (READBACK_ENABLED)
    Serial.println("Ready.");
}

// TODO: Panic if no serial input recieved for N seconds

void loop() {
  // Update controls based on serial input
  if (Serial.available()) {
    String command = Serial.readString();

    if (command.startsWith("<") && command.endsWith("/>")){
      int steerIdx = command.indexOf("s");
      int throttleIdx = command.indexOf("t");
      int endIdx = command.indexOf("/");
      steering = command.substring(steerIdx + 1, throttleIdx).toInt();
      throttle = command.substring(throttleIdx + 1, endIdx).toInt();
    }
    
    if (READBACK_ENABLED) {
      Serial.print("S: ");
      Serial.println(steering);
      Serial.print("T: ");
      Serial.println(throttle);
    }
  }

  if (MOTOR_ENABLED)
    motor_servo.writeMicroseconds(
      map(throttle, 0, 100, 1500, 1800)
    );
  if (STEER_ENABLED)
    steer_servo.write(steering);
}
