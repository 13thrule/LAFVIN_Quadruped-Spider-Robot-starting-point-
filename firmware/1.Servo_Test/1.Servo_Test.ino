#include <Servo.h>

Servo myservo;
#define SERVO_PIN D4

void setup() {
 myservo.attach(SERVO_PIN);
 myservo.write(0);
}

void loop() {
 myservo.write(180);
 delay(3000);

 myservo.write(0);
 delay(3000);
}