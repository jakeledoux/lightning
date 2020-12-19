#include <Servo.h>

#define PANIC_INTERVAL 5000
#define MOTOR_PIN 3
#define STEER_PIN 5
#define BUZZER_PIN 11
#define MOTOR_ENABLED true
#define STEER_ENABLED true
#define BUZZER_ENABLED true
#define READBACK_ENABLED false

Servo motor_servo;
Servo steer_servo;
int steering = 90;
int throttle = 0;
int buzzer = 0;

int iterSinceLastCommand = 0;

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

// Neutral command: <s90t0b0/>
void loop() {
  // Update controls based on serial input
  if (Serial.available()) {
    String command = Serial.readString();
  
    if (command.startsWith("<") && command.endsWith("/>")){
      iterSinceLastCommand = 0;
      int steerIdx = command.indexOf("s");
      int throttleIdx = command.indexOf("t");
      int buzzIdx = command.indexOf("b");
      int endIdx = command.indexOf("/");
      steering = command.substring(steerIdx + 1, throttleIdx).toInt();
      throttle = command.substring(throttleIdx + 1, buzzIdx).toInt();
      buzzer = command.substring(buzzIdx + 1, endIdx).toInt();
    }
    
    if (READBACK_ENABLED) {
      Serial.print("S: ");
      Serial.println(steering);
      Serial.print("T: ");
      Serial.println(throttle);
    }
  }
  
  if (iterSinceLastCommand++ > PANIC_INTERVAL) {
      iterSinceLastCommand = 0;
      steering = 90;
      throttle = 0;
  }

  if (MOTOR_ENABLED)
    motor_servo.writeMicroseconds(
      map(throttle, 0, 100, 1500, 1800)
    );
  if (STEER_ENABLED)
    steer_servo.write(steering);
  if (BUZZER_ENABLED && buzzer == 1) {
    tone(BUZZER_PIN, 440, 30);
    buzzer = 0;
  }
}
