#include <Servo.h>

Servo myservo1;  // Pin D4 SERVO
Servo myservo2;  // Pin D8 SERVO

#define SERVO_PIN1 D4
#define SERVO_PIN2 D8

void setup() {
 myservo1.attach(SERVO_PIN1);
 myservo2.attach(SERVO_PIN2);

 myservo1.write(0);
 myservo2.write(0);
}

void loop() {
 // Both servos rotate 180 degrees simultaneously

 myservo1.write(180);
 myservo2.write(180);
 delay(2000);

 // Both servos return to 0 degrees simultaneously

 myservo1.write(0);
 myservo2.write(0);
 delay(2000);
}