#include <Servo.h>
#include <EEPROM.h>
#include <IRrecv.h>
#include <IRutils.h>
#include "spiderbot_motion.h"
#include "webserver.h"

// IR receiver pin differs between the ESP32-C3 test target and the ESP8266 robot board.
#ifdef ARDUINO_ESP32C3_DEV
const uint16_t kRecvPin = 10;
#else
const uint16_t kRecvPin = D3;
#endif

IRrecv irrecv(kRecvPin);
decode_results results;

uint32_t irnum = 0;

// Last program id selected by a real (non-repeat) IR keypress. NEC remotes
// send a distinct zero-data "repeat" frame roughly every 110ms while a
// button stays held, rather than resending the original code, so we need
// this to know what to keep running when a repeat frame arrives.
static int lastIrProgramId = PROGRAM_NONE;

// Avoidance distance threshold in centimeters.
#define AVOIDDIS 40
#define ENABLE_DEBUG_SERIAL 0

// The current avoidance wiring uses TX as TRIG and RX as ECHO, so the
// ultrasonic sensor and the USB serial port share the same two physical
// pins. They cannot be used at the same time -- and it's not just a
// physical-connection thing: pinMode(TRIG_PIN, OUTPUT) below reprograms
// GPIO1's pin mux away from the UART-TX function regardless of whether a
// sensor is actually wired to it, which silences Serial output (including
// spiderbot_motion.cpp's POS: telemetry) even with the sensor unplugged.
// Set this to 0 to skip claiming the ultrasonic pins at boot and get a
// working serial link (telemetry, calibration); back to 1 for normal
// obstacle-avoidance operation. Physically unplug the ultrasonic sensor
// whenever this is 0, same as this kit's own flashing instructions require.
#define ENABLE_ULTRASONIC 0
static const uint8_t ULTRASONIC_TRIG_PIN = 1;  // TX
static const uint8_t ULTRASONIC_ECHO_PIN = 3;  // RX
static const unsigned long ULTRASONIC_TIMEOUT_US = 30000;

// Serial is needed for SpiderBotMotion::calibration() regardless of whether
// verbose debug logging is enabled, so bring the UART up unconditionally.
// ENABLE_DEBUG_SERIAL only controls how chatty it is.
#define DEBUG_SERIAL_BEGIN(baud) Serial.begin(baud)

#if ENABLE_DEBUG_SERIAL
#define DEBUG_PRINT(value) Serial.print(value)
#define DEBUG_PRINTLN(value) Serial.println(value)
#define DEBUG_PRINT_HEX64(value) serialPrintUint64((value), HEX)
#else
#define DEBUG_PRINT(value) do {} while (0)
#define DEBUG_PRINTLN(value) do {} while (0)
#define DEBUG_PRINT_HEX64(value) do {} while (0)
#endif

float distance = 0;

// Main robot motion object plus a few local helpers used by setup/loop.
SpiderBotMotion robot;
static int decodeIrProgram(uint32_t code);
static void runRobotProgram(int programId);
static float measureAvoidDistanceCm();

// Only locomotion commands make sense to chain continuously while a button
// stays held -- poses/performance moves (hello, dances, etc.) still run
// once per keypress.
static bool isHoldableLocomotion(int programId) {
  switch (programId) {
    case PROGRAM_FORWARD:
    case PROGRAM_BACKWARD:
    case PROGRAM_LEFTSHIFT:
    case PROGRAM_RIGHTSHIFT:
    case PROGRAM_TURNLEFT:
    case PROGRAM_TURNRIGHT:
      return true;
    default:
      return false;
  }
}

// Board startup order:
// 1. bring up debug / IR / ultrasonic pins
// 2. start Wi-Fi + web server
// 3. attach servos and move to a known pose
void setup()
{
  DEBUG_SERIAL_BEGIN(9600);
  irrecv.enableIRIn();
#if ENABLE_ULTRASONIC
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
#endif
  DEBUG_PRINTLN("QuadBot-E Start!");
  delay(1000);

  webinit();
  robot.init();
  robot.Servo_PROGRAM_Zero();
  enableWebServer();
}

// Main loop responsibilities:
// 1. consume IR commands
// 2. serve HTTP requests
// 3. read per-servo calibration commands off serial (see calibration()'s
//    "S,<servo_pin>,<offset>" format; requires the ultrasonic sensor
//    disconnected, since it shares the TX/RX pins)
// 4. execute queued web/API actions
// 5. apply direct single-servo debug commands
void loop()
{
  bool haveIrResult = irrecv.decode(&results);

  // A while, not an if: if a locomotion hold below gets interrupted by a
  // genuinely new (non-repeat) keypress, that decode is reprocessed here
  // as a fresh command instead of being silently dropped.
  while (haveIrResult) {
    haveIrResult = false;
    int programId = PROGRAM_NONE;

    DEBUG_PRINT_HEX64(results.value);
    delay(150);
    irrecv.resume();

    if (results.repeat) {
      // The button is still held down; re-run whatever the last real
      // keypress selected so holding forward/turn/etc. walks continuously
      // instead of capping every physical press at one stride.
      programId = lastIrProgramId;
    } else {
      irnum = (uint32_t)results.value;
      programId = decodeIrProgram(irnum);
    }

    if (programId != PROGRAM_NONE) {
      DEBUG_PRINTLN(getActionLabel(programId));
      runRobotProgram(programId);
      lastIrProgramId = programId;

      // While a locomotion button stays held, keep striding back-to-back
      // without routing back through the top of loop() each time -- that
      // path re-applies the 150ms decode debounce on every repeat frame,
      // which turned "hold to walk" into a chain of separately-paused
      // single steps instead of one continuous walk.
      while (isHoldableLocomotion(programId) && !isRobotStopRequested()) {
        if (!irrecv.decode(&results)) break;

        if (results.repeat) {
          irrecv.resume();
          runRobotProgram(programId);
          continue;
        }

        // A distinct new keypress arrived mid-hold (e.g. switching
        // direction). Stop this chain and let the top of this block
        // process it as its own command instead of dropping it.
        haveIrResult = true;
        break;
      }
    }

    DEBUG_PRINTLN(irnum);
    irnum = 0;
  }

  handleClient();
  robot.calibration();

  if (Servo_PROGRAM >= 1) {
    DEBUG_PRINTLN(getActionLabel(Servo_PROGRAM));
    runRobotProgram(Servo_PROGRAM);
    Servo_PROGRAM = 0;
  }

  if (GPIO_ID < 100) {
    DEBUG_PRINT("GPIO_ID=");
    DEBUG_PRINTLN(GPIO_ID);
    DEBUG_PRINT("ival=");
    DEBUG_PRINTLN(ival);
    robot.Set_PWM_to_Servo(GPIO_ID, ival.toInt());
    GPIO_ID = 100;
    ival = "";
  }
}

// Trigger a standard HC-SR04 style measurement and convert it to centimeters.
// Returns -1 when no echo is received before timeout.
static float measureAvoidDistanceCm()
{
#if ENABLE_ULTRASONIC
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  const unsigned long durationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (durationUs == 0) {
    return -1.0f;
  }

  return durationUs / 58.0f;
#else
  // TX/RX are serving as the serial link right now (see ENABLE_ULTRASONIC),
  // not the ultrasonic sensor -- driving them here would corrupt whatever
  // is being transmitted, so just report "no echo" instead.
  return -1.0f;
#endif
}

// Blocking avoidance mode:
// keep moving forward while the path is clear, otherwise stop and turn left.
// It exits when a stop request arrives, when the IR exit key is pressed,
// or when Servo_PROGRAM is set to the legacy break value 22.
void Avoid(void)
{
  while (1) {
    distance = measureAvoidDistanceCm();

    if (distance < AVOIDDIS && distance > 0) {
      robot.center();
      delay(100);
      robot.turnleft();
      robot.turnleft();
    } else if (distance >= AVOIDDIS) {
      robot.forward();
    }

    handleClient();

    if (isRobotStopRequested()) {
      break;
    }

    if (irrecv.decode(&results)) {
      DEBUG_PRINT_HEX64(results.value);
      irnum = (uint32_t)results.value;
      delay(150);
      irrecv.resume();
      if (irnum == 0xFF02FD) {
        break;
      }
    }

    if (Servo_PROGRAM == 22) {
      break;
    }
  }

  delay(1000);
}


// Legacy mapping kept here as a reference for the older unnamed IR remote.
/* static int decodeIrProgram(uint32_t code)
{
  switch (code)
  {
    case 0xFF18E7:
      return PROGRAM_FORWARD; // 上
    case 0xFF4AB5:
      return PROGRAM_BACKWARD; // 下
    case 0xFF10EF:
      return PROGRAM_LEFTSHIFT; // 左
    case 0xFF5AA5:
      return PROGRAM_RIGHTSHIFT; // 右
    case 0xFF38C7:
      return PROGRAM_ZERO; // OK
    case 0xFFB04F:
      return PROGRAM_CENTER; // #
    case 0xFF6897:
      return PROGRAM_SLEEP; //  *
    case 0xFFA25D:
      return PROGRAM_TURNLEFT; // 1
    case 0xFF629D:
      return PROGRAM_TURNRIGHT; // 2
    case 0xFFE21D:
      return PROGRAM_LIE; // 3
    case 0xFF22DD:
      return PROGRAM_HELLO; // 4
    case 0xFF02FD:
      return PROGRAM_FIGHTING; // 5
    case 0xFFC23D:
      return PROGRAM_PUSHUP; // 6
    case 0xFFE01F:
      return PROGRAM_DANCE1; // 7
    case 0xFFA857:
      return PROGRAM_DANCE2; // 8
    case 0xFF906F:
      return PROGRAM_DANCE3; // 9
    case 0xFF9867:
      return PROGRAM_AVOID; // 0
    default:
      return PROGRAM_NONE;
  }
} */
// Active IR mapping for the current remote controller.
// This converts raw IR values into the shared RobotProgramId enum.
static int decodeIrProgram(uint32_t code)
{
  switch (code)
  {
    case 0xFF629D:
      return PROGRAM_FORWARD;// ok 
    case 0xFFA857:
      return PROGRAM_BACKWARD;// ok
    case 0xFF22DD:
      return PROGRAM_LEFTSHIFT;// ok
    case 0xFFC23D:
      return PROGRAM_RIGHTSHIFT;// ok
    case 0xFF02FD:
      return PROGRAM_ZERO;// ok
    case 0xFF52AD:
      return PROGRAM_CENTER;// ok
    case 0xFF42BD:
      return PROGRAM_SLEEP;// ok
    case 0xFF6897:
      return PROGRAM_TURNLEFT; // ok
    case 0xFF9867:
      return PROGRAM_TURNRIGHT; // ok
    case 0xFFB04F:
      return PROGRAM_LIE;// ok
    case 0xFF30CF:
      return PROGRAM_HELLO;// ok
    case 0xFF18E7:
      return PROGRAM_FIGHTING;// ok
    case 0xFF7A85:
      return PROGRAM_PUSHUP;// ok
    case 0xFF10EF:
      return PROGRAM_DANCE1;// ok
    case 0xFF38C7:
      return PROGRAM_DANCE2;// ok
    case 0xFF5AA5:
      return PROGRAM_DANCE3;// ok
    case 0xFF4AB5:
      return PROGRAM_AVOID;
    default:
      return PROGRAM_NONE;
  }
}

// Single dispatch point used by IR, Web UI and API commands.
// Every control source eventually resolves to one RobotProgramId and lands here.
static void runRobotProgram(int programId)
{
  setCurrentAction(programId, true);

  switch (programId)
  {
    case PROGRAM_STANDBY:
      robot.standby();
      break;
    case PROGRAM_FORWARD:
      robot.forward();
      break;
    case PROGRAM_BACKWARD:
      robot.backward();
      break;
    case PROGRAM_LEFTSHIFT:
      robot.leftshift();
      break;
    case PROGRAM_RIGHTSHIFT:
      robot.rightshift();
      break;
    case PROGRAM_TURNLEFT:
      robot.turnleft();
      break;
    case PROGRAM_TURNRIGHT:
      robot.turnright();
      break;
    case PROGRAM_LIE:
      robot.lie();
      break;
    case PROGRAM_HELLO:
      robot.hello();
      break;
    case PROGRAM_FIGHTING:
      robot.fighting();
      break;
    case PROGRAM_PUSHUP:
      robot.pushup();
      break;
    case PROGRAM_SLEEP:
      robot.sleep();
      break;
    case PROGRAM_DANCE1:
      robot.dance1();
      break;
    case PROGRAM_DANCE2:
      robot.dance2();
      break;
    case PROGRAM_DANCE3:
      robot.dance3();
      break;
    case PROGRAM_AVOID:
      Avoid();
      break;
    case PROGRAM_CENTER:
      robot.center();
      break;
    case PROGRAM_ZERO:
      robot.zero();
      break;
    default:
      break;
  }

  if (isRobotStopRequested()) {
    int stopTargetProgram = getRobotStopTargetProgram();
    clearRobotStopRequest();

    if (stopTargetProgram == PROGRAM_CENTER) {
      robot.center();
    } else if (stopTargetProgram == PROGRAM_ZERO) {
      robot.zero();
    } else {
      robot.standby();
    }
  }

  clearCurrentAction();
}
