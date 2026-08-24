#include <ESP8266WiFi.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <Servo.h>

// ===== Pin Definitions =====
#define IR_RECEIVE_PIN D3
#define SERVO1_PIN D4
#define SERVO2_PIN D8

// ===== IR Key Mapping =====
#define KEY_1 0x16
#define KEY_2 0x19

// ===== Servo Objects =====
Servo servo1;
Servo servo2;

// ===== IR Receiver Object =====
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;

// ===== Key Mapping Function =====
String getKeyName(uint32_t code) {
  switch(code) {
    case KEY_1: return "1";
    case KEY_2: return "2";
    default: return "";
  }
}

// ===== Control Servo Function =====
void controlServo(String key) {
  if (key == "1") {
    servo1.write(180);
    delay(500);
    servo1.write(0);
  } 
  else if (key == "2") {
    servo2.write(180);
    delay(500);
    servo2.write(0);
  }
}

// ===== Setup =====
void setup() {
  // Attach servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  
  // Initialize servos to 0 degrees
  servo1.write(0);
  servo2.write(0);
  
  // Initialize IR receiver
  irrecv.enableIRIn();
}

// ===== Main Loop =====
void loop() {
  // Detect IR signals
  if (irrecv.decode(&results)) {
    uint32_t command = results.command;
    String key = getKeyName(command);
    
    // Try using value's lower 8 bits if command mapping fails
    if (key == "") {
      key = getKeyName(results.value & 0xFF);
    }
    
    if (key != "") {
      controlServo(key);
      delay(100);
    }
    
    irrecv.resume();
  }
}