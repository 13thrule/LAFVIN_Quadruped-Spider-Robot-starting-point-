#ifndef actionarray_h
#define actionarray_h

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// Servo matrix
const int PWMRES_Min = 1;
const int PWMRES_Max = 180;
const int ALLMATRIX = 9; // 8 servos + runtime
const int ALLSERVOS = 8;
const int SERVOMIN = 400;
const int SERVOMAX = 2400;

// Servo offset angle
// ------------------ G14, G12, G13, G15, G16, G5,  G4,  G2
const int Servo_Offset[] PROGMEM = { -2, -9, -5, -3, 0, -12, 5, 5 };

// Servo zero position
// ------------------- G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
const int Servo_Act_0[] PROGMEM = { 135, 45, 135, 45, 45, 135, 45, 135, 500 };

// Start / standby position
// ------------------------ G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
const int Servo_Act_1[] PROGMEM = { 60, 90, 90, 120, 120, 90, 90, 60, 500 };

// Standby
const int Servo_Prg_1_Step = 2;
const int Servo_Prg_1[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  90,  90,  90,  90,  90,  90,  90,  90, 500 }, // servo center point
  {  60,  90,  90, 120, 120,  90,  90,  60, 500 }, // standby
};

// Forward -- statically-stable wave/crawl walk (one leg lifts at a time:
// RF -> LF -> LR -> RR, three feet always grounded). The lunge/frog-hop
// tried here didn't feel right on real hardware and was dropped -- this
// is the same geometry as the previously-preserved Servo_Prg_16 below,
// just retuned from 200ms/step to 90ms/step and re-validated in
// sim_physics/observe.py: speed went from 1.3mm/s to 5.1mm/s (4x) with
// stability essentially unchanged (max tilt 9.3deg -> 8.4deg over 3
// cycles, stayed upright both times). 90ms/step is close to the
// loaded-MG90S ~2.5ms/degree tracking floor from earlier research for
// this gait's largest per-step angle changes (up to ~45deg), so this is
// close to as fast as this gait can go without commanding faster than
// the servos can actually track.
const int Servo_Prg_2_Step = 16;
const int Servo_Prg_2[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  75, 110, 110, 105, 105,  70,  90 }, // RF turn: shift RR/LF/LR
  {  90,  90,  75, 110, 110, 105, 105,  70,  90 }, // RF foot up
  {  90, 135,  75, 110, 110, 105, 105,  70,  90 }, // RF swing recovers
  {  70, 135,  75, 110, 110, 105, 105,  70,  90 }, // RF foot down

  {  70, 120,  60, 110, 110, 105, 120,  70,  90 }, // LF turn: shift RF/RR/LR
  {  70, 120,  60, 110,  90, 105, 120,  70,  90 }, // LF foot up
  {  70, 120,  60, 110,  90,  60, 120,  70,  90 }, // LF swing recovers
  {  70, 120,  60, 110, 110,  60, 120,  70,  90 }, // LF foot down

  {  70, 105,  45, 110, 110,  75, 120,  70,  90 }, // LR turn: shift RF/RR/LF
  {  70, 105,  45, 110, 110,  75, 120,  90,  90 }, // LR foot up
  {  70, 105,  45, 110, 110,  75,  75,  90,  90 }, // LR swing recovers
  {  70, 105,  45, 110, 110,  75,  75,  70,  90 }, // LR foot down

  {  70,  90,  45, 110, 110,  90,  90,  70,  90 }, // RR turn: shift RF/LF/LR
  {  70,  90,  45,  90, 110,  90,  90,  70,  90 }, // RR foot up
  {  70,  90,  90,  90, 110,  90,  90,  70,  90 }, // RR swing recovers
  {  70,  90,  90, 110, 110,  90,  90,  70,  90 }, // RR foot down -- back to start pose
};

// Backward
const int Servo_Prg_3_Step = 11;
const int Servo_Prg_3[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 200 }, // standby
  {  90,  45,  90, 110, 110,  90,  90,  90, 200 }, // leg4,1 up; leg1 fw
  {  70,  45,  90, 110, 110,  90,  90,  70, 200 }, // leg4,1 down
  {  70,  45,  90,  90,  90,  90,  90,  70, 200 }, // leg3,2 up
  {  70,  90,  90,  90,  90, 135,  45,  70, 200 }, // leg4,1 back; leg3 fw
  {  70,  90,  90, 110, 110, 135,  45,  70, 200 }, // leg3,2 down
  {  90,  90,  90, 110, 110, 135,  90,  90, 200 }, // leg4,1 up; leg4 fw
  {  90,  90, 135, 110, 110,  90,  90,  90, 200 }, // leg3,1 back
  {  70,  90, 135, 110, 110,  90,  90,  70, 200 }, // leg4,1 down
  {  70,  90, 135,  90, 110,  90,  90,  70, 200 }, // leg2 up
  {  70,  90,  90, 110, 110,  90,  90,  70, 200 }, // leg2 fw down
};

// Left shift
const int Servo_Prg_4_Step = 11;
const int Servo_Prg_4[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 200 }, // standby
  {  70,  90,  45,  90,  90,  90,  90,  70, 200 }, // leg3,2 up; leg2 fw
  {  70,  90,  45, 110, 110,  90,  90,  70, 200 }, // leg3,2 down
  {  90,  90,  45, 110, 110,  90,  90,  90, 200 }, // leg1,4 up
  {  90, 135,  90, 110, 110,  45,  90,  90, 200 }, // leg3,2 back; leg1 fw
  {  70, 135,  90, 110, 110,  45,  90,  70, 200 }, // leg1,4 down
  {  70, 135,  90,  90,  90,  90,  90,  70, 200 }, // leg3,2 up; leg3 fw
  {  70,  90,  90,  90,  90,  90, 135,  70, 200 }, // leg1,4 back
  {  70,  90,  90, 110, 110,  90, 135,  70, 200 }, // leg3,2 down
  {  70,  90,  90, 110, 110,  90, 135,  90, 200 }, // leg4 up
  {  70,  90,  90, 110, 110,  90,  90,  70, 200 }, // leg4 fw down
};

// Right shift
const int Servo_Prg_5_Step = 11;
const int Servo_Prg_5[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 200 }, // standby
  {  70,  90,  90,  90,  90,  45,  90,  70, 200 }, // leg2,3 up; leg3 fw
  {  70,  90,  90, 110, 110,  45,  90,  70, 200 }, // leg2,3 down
  {  90,  90,  90, 110, 110,  45,  90,  90, 200 }, // leg4,1 up
  {  90,  90,  45, 110, 110,  90, 135,  90, 200 }, // leg2,3 back; leg4 fw
  {  70,  90,  45, 110, 110,  90, 135,  70, 200 }, // leg4,1 down
  {  70,  90,  90,  90,  90,  90, 135,  70, 200 }, // leg2,3 up; leg2 fw
  {  70, 135,  90,  90,  90,  90,  90,  70, 200 }, // leg4,1 back
  {  70, 135,  90, 110, 110,  90,  90,  70, 200 }, // leg2,3 down
  {  90, 135,  90, 110, 110,  90,  90,  70, 200 }, // leg1 up
  {  70,  90,  90, 110, 110,  90,  90,  70, 200 }, // leg1 fw down
};

// Turn left
const int Servo_Prg_6_Step = 8;
const int Servo_Prg_6[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 200 }, // standby
  {  90,  90,  90, 110, 110,  90,  90,  90, 200 }, // leg1,4 up
  {  90, 135,  90, 110, 110,  90, 135,  90, 200 }, // leg1,4 turn
  {  70, 135,  90, 110, 110,  90, 135,  70, 200 }, // leg1,4 down
  {  70, 135,  90,  90,  90,  90, 135,  70, 200 }, // leg2,3 up
  {  70, 135, 135,  90,  90, 135, 135,  70, 200 }, // leg2,3 turn
  {  70, 135, 135, 110, 110, 135, 135,  70, 200 }, // leg2,3 down
  {  70,  90,  90, 110, 110,  90,  90,  70, 200 }, // leg1,2,3,4 turn
};

// Turn right
const int Servo_Prg_7_Step = 8;
const int Servo_Prg_7[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 200 }, // standby
  {  70,  90,  90,  90,  90,  90,  90,  70, 200 }, // leg2,3 up
  {  70,  90,  45,  90,  90,  45,  90,  70, 200 }, // leg2,3 turn
  {  70,  90,  45, 110, 110,  45,  90,  70, 200 }, // leg2,3 down
  {  90,  90,  45, 110, 110,  45,  90,  90, 200 }, // leg1,4 up
  {  90,  45,  45, 110, 110,  45,  45,  90, 200 }, // leg1,4 turn
  {  70,  45,  45, 110, 110,  45,  45,  70, 200 }, // leg1,4 down
  {  70,  90,  90, 110, 110,  90,  90,  70, 200 }, // leg1,2,3,4 turn
};

// Lie
const int Servo_Prg_8_Step = 1;
const int Servo_Prg_8[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  { 110,  90,  90,  70,  70,  90,  90, 110, 500 }, // leg1,4 up
};

// Say Hi / Greeting wave
// Servo order:
// 0:G14=Right front foot, 1:G12=Right front leg
// 2:G13=Right rear leg,  3:G15=Right rear foot
// 4:G16=Left front foot, 5:G5=Left front leg
// 6:G4=Left rear leg,    7:G2=Left rear foot
const int Servo_Prg_9_Step = 11;
const int Servo_Prg_9[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 300 }, // re-establish the standing pose this action assumes
  {  60,  90,  125, 80, 120,  82, 90,  50, 280 }, // lean into a 3-leg support pose
  {  60,  90,  125, 80,  92,  74, 90,  50, 220 }, // left hand halfway up
  {  60,  90,  125, 80,  38,  62, 90,  50, 240 }, // left hand fully up
  {  60,  90,  125, 80,  34, 118, 90,  50, 260 }, // wave outward
  {  60,  90,  125, 80,  46,  58, 90,  50, 260 }, // wave inward
  {  60,  90,  125, 80,  34, 118, 90,  50, 260 }, // wave outward
  {  60,  90,  125, 80,  46,  58, 90,  50, 260 }, // wave inward
  {  60,  90,  125, 80,  44,  88, 90,  50, 220 }, // hand returns to center
  {  60,  90,  125, 80,  92,  78, 90,  50, 220 }, // lower halfway, keep body tilted
  {  60,  90,  125, 80, 120,  82, 90,  50, 280 }, // cute tilted finish pose
};

// Fighting
const int Servo_Prg_10_Step = 13;
const int Servo_Prg_10[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 300 }, // re-establish the standing pose this action assumes
  { 120,  90,  90, 110,  60,  90,  90,  70, 500 }, // leg1,2 down
  { 120,  70,  70, 110,  60,  70,  70,  70, 500 }, // body turn left
  { 120, 110, 110, 110,  60, 110, 110,  70, 500 }, // body turn right
  { 120,  70,  70, 110,  60,  70,  70,  70, 500 }, // body turn left
  { 120, 110, 110, 110,  60, 110, 110,  70, 500 }, // body turn right
  {  70,  90,  90,  70, 110,  90,  90, 110, 500 }, // leg1,2 up; leg3,4 down
  {  70,  70,  70,  70, 110,  70,  70, 110, 500 }, // body turn left
  {  70, 110, 110,  70, 110, 110, 110, 110, 500 }, // body turn right
  {  70,  70,  70,  70, 110,  70,  70, 110, 500 }, // body turn left
  {  70, 110, 110,  70, 110, 110, 110, 110, 500 }, // body turn right
  {  70,  90,  90,  70, 110,  90,  90, 110, 500 }, // leg1,2 up; leg3,4 down
  {  60,  90,  90, 120, 120,  90,  90,  60, 500 },
};

// Push up
const int Servo_Prg_11_Step = 14;
const int Servo_Prg_11[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 300 }, // re-establish the standing pose this action assumes
  {  70,  90,  90, 120, 120,  90,  90,  70,  500 }, // start
  { 100,  90,  90,  80,  80,  90,  90, 100,  600 }, // down
  {  60,  90,  90, 120, 120,  90,  90,  60,  700 }, // up
  { 100,  90,  90,  80,  80,  90,  90, 100,  800 }, // down
  {  60,  90,  90, 120, 120,  90,  90,  60,  900 }, // up
  { 100,  90,  90,  80,  80,  90,  90, 100,  1000 }, // down
  {  60,  90,  90, 120, 120,  90,  90,  60,  1100 }, // up
  { 100,  90,  90,  80,  80,  90,  90, 100, 2000 }, // down
  {  60,  90,  90, 120, 120,  90,  90,  60, 2500 }, // up
  { 135,  90,  90,  45,  45,  90,  90, 135,  200 }, // fast down
  {  70,  90,  90,  45,  60,  90,  90, 135,  800 }, // leg1 up
  {  70,  90,  90,  45, 110,  90,  90, 135,  800 }, // leg2 up
  {  70,  90,  90, 110, 110,  90,  90,  70,  800 }, // leg3,4 up
};

// Sleep
const int Servo_Prg_12_Step = 2;
const int Servo_Prg_12[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  30,  90,  90, 150, 150,  90,  90,  30, 500 }, // leg1,4 down
  {  30,  45, 135, 150, 150, 135,  45,  30, 500 }, // protect myself
};

// Dance 1
const int Servo_Prg_13_Step = 11;
const int Servo_Prg_13[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 300 }, // re-establish the standing pose this action assumes
  {  90,  90,  90,  90,  90,  90,  90,  90, 400 }, // leg1,2,3,4 up
  {  50,  90,  90,  90,  90,  90,  90,  90, 400 }, // leg1 down
  {  90,  90,  90, 130,  90,  90,  90,  90, 400 }, // leg1 up; leg2 down
  {  90,  90,  90,  90,  90,  90,  90,  50, 400 }, // leg2 up; leg4 down
  {  90,  90,  90,  90, 130,  90,  90,  90, 400 }, // leg4 up; leg3 down
  {  50,  90,  90,  90,  90,  90,  90,  90, 400 }, // leg3 up; leg1 down
  {  90,  90,  90, 130,  90,  90,  90,  90, 400 }, // leg1 up; leg2 down
  {  90,  90,  90,  90,  90,  90,  90,  50, 400 }, // leg2 up; leg4 down
  {  90,  90,  90,  90, 130,  90,  90,  90, 400 }, // leg4 up; leg3 down
  {  90,  90,  90,  90,  90,  90,  90,  90, 400 }, // leg3 up
};

// Dance 2
const int Servo_Prg_14_Step = 11;
const int Servo_Prg_14[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 300 }, // re-establish the standing pose this action assumes
  {  70,  45, 135, 110, 110, 135,  45,  70, 320 }, // sesame-style dance stance
  {  95,  45, 135,  85, 110, 135,  45,  70, 260 }, // right side softens inward
  {  70,  45, 135, 110,  85, 135,  45,  95, 260 }, // left side softens inward
  {  95,  45, 135,  85, 110, 135,  45,  70, 260 }, // right side softens inward
  {  70,  45, 135, 110,  85, 135,  45,  95, 260 }, // left side softens inward
  {  95,  45, 135,  85, 110, 135,  45,  70, 260 }, // right side softens inward
  {  70,  45, 135, 110,  85, 135,  45,  95, 260 }, // left side softens inward
  {  95,  45, 135,  85, 110, 135,  45,  70, 260 }, // right side softens inward
  {  70,  45, 135, 110,  85, 135,  45,  95, 260 }, // left side softens inward
  {  70,  45, 135, 110, 110, 135,  45,  70, 320 }, // settle in a balanced dance stance
};

// Dance 3
const int Servo_Prg_15_Step = 11;
const int Servo_Prg_15[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 300 }, // re-establish the standing pose this action assumes
  {  70,  45,  45, 110, 110, 135, 135,  70, 400 }, // leg1,2,3,4 back
  { 110,  45,  45,  60,  70, 135, 135,  70, 400 }, // leg1,2,3 up
  {  70,  45,  45, 110, 110, 135, 135,  70, 400 }, // leg1,2,3 down
  { 110,  45,  45, 110,  70, 135, 135, 120, 400 }, // leg1,3,4 up
  {  70,  45,  45, 110, 110, 135, 135,  70, 400 }, // leg1,3,4 down
  { 110,  45,  45,  60,  70, 135, 135,  70, 400 }, // leg1,2,3 up
  {  70,  45,  45, 110, 110, 135, 135,  70, 400 }, // leg1,2,3 down
  { 110,  45,  45, 110,  70, 135, 135, 120, 400 }, // leg1,3,4 up
  {  70,  45,  45, 110, 110, 135, 135,  70, 400 }, // leg1,3,4 down
  {  70,  90,  90, 110, 110,  90,  90,  70, 400 }, // standby
};

// Preserved, currently unused: the statically-stable wave/crawl gait that
// used to be "forward" (Servo_Prg_2) before that slot became the frog-hop.
// One leg lifts at a time, the other three stay grounded -- the center of
// mass stays inside the support triangle of 3 grounded feet at every
// instant, by geometry alone, unlike a trot (only dynamically stable, 2
// feet grounded mid-swing). Kept here, not deleted, in case a genuine
// walking gait is wanted again later; nothing currently calls this.
const int Servo_Prg_16_Step = 16;
const int Servo_Prg_16[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  75, 110, 110, 105, 105,  70, 200 }, // RF turn: shift RR/LF/LR
  {  90,  90,  75, 110, 110, 105, 105,  70, 200 }, // RF foot up
  {  90, 135,  75, 110, 110, 105, 105,  70, 200 }, // RF swing recovers
  {  70, 135,  75, 110, 110, 105, 105,  70, 200 }, // RF foot down

  {  70, 120,  60, 110, 110, 105, 120,  70, 200 }, // LF turn: shift RF/RR/LR
  {  70, 120,  60, 110,  90, 105, 120,  70, 200 }, // LF foot up
  {  70, 120,  60, 110,  90,  60, 120,  70, 200 }, // LF swing recovers
  {  70, 120,  60, 110, 110,  60, 120,  70, 200 }, // LF foot down

  {  70, 105,  45, 110, 110,  75, 120,  70, 200 }, // LR turn: shift RF/RR/LF
  {  70, 105,  45, 110, 110,  75, 120,  90, 200 }, // LR foot up
  {  70, 105,  45, 110, 110,  75,  75,  90, 200 }, // LR swing recovers
  {  70, 105,  45, 110, 110,  75,  75,  70, 200 }, // LR foot down

  {  70,  90,  45, 110, 110,  90,  90,  70, 200 }, // RR turn: shift RF/LF/LR
  {  70,  90,  45,  90, 110,  90,  90,  70, 200 }, // RR foot up
  {  70,  90,  90,  90, 110,  90,  90,  70, 200 }, // RR swing recovers
  {  70,  90,  90, 110, 110,  90,  90,  70, 200 }, // RR foot down -- back to start pose
};

// Sit: rear feet tuck up under the body (RR/LR knees driven toward their
// lift/raise direction), front feet stay planted at the standing pose --
// a real sit silhouette (haunches down, front up), not just "lie" (which
// is a single diagonal-pair tilt, not a sit). Hips stay centered.
// Sim-validated (sim_physics/observe.py): stayed upright, max tilt 8.0deg.
// Self-contained: the first row re-establishes the 70/110 standing pose
// this whole action was designed from, regardless of whatever pose the
// robot was actually in when this was triggered (found the hard way --
// PROGRAM_STANDBY settles at a different 60/120 pose, not 70/110, so
// chaining these off of "standby" silently halved bow's dip and leaked
// unintended rear-leg motion the first time this was tested for real).
const int Servo_Prg_17_Step = 2;
const int Servo_Prg_17[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 300 }, // re-establish the standing pose this action assumes
  {  70,  90,  90,  70, 110,  90,  90, 110, 500 }, // settle into sit -- rear knees tuck up, front stays standing
};

// Bow: front feet extend down while rear feet hold the standing pose --
// dips the front of the body down and back up. First tuned to a 20deg
// dip (50/130) at 350ms and reported as barely visible on real hardware --
// not a stale-position bug (the interpolator does reach its target
// regardless of starting pose, confirmed by reading Servo_PROGRAM_Run
// directly), just too small and fast to read as a deliberate bow. Widened
// to 35/145, still reported as needing to go deeper -- now pushed all the
// way to 30/150, the exact same knee extreme sleep already uses safely on
// real hardware (not going past a value already proven there).
// Self-contained -- see Servo_Prg_17's comment above for why the leading
// standing-pose row matters.
const int Servo_Prg_18_Step = 4;
const int Servo_Prg_18[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 300 }, // re-establish the standing pose this action assumes
  {  30,  90,  90, 110, 150,  90,  90,  70, 450 }, // dip -- front knees extend down, rear unchanged
  {  30,  90,  90, 110, 150,  90,  90,  70, 400 }, // hold the bow
  {  70,  90,  90, 110, 110,  90,  90,  70, 450 }, // recover -- back to standing
};

// Shake / wiggle: a small-amplitude hip twist combined with all 4 knees
// bouncing together (compress on one beat, extend on the other) --
// deliberately different from dance2, which only moves knees, and from
// the original hip-only version of this, which didn't move the knees at
// all. A body wiggle/shimmy in place, not a locomotion gait.
// Sim-validated: stayed upright, max tilt 1.8deg. Self-contained -- see
// Servo_Prg_17's comment above for why the leading standing-pose row
// matters.
const int Servo_Prg_19_Step = 6;
const int Servo_Prg_19[][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  70,  90,  90, 110, 110,  90,  90,  70, 300 }, // re-establish the standing pose this action assumes
  {  60, 105,  75, 120, 120,  75, 105,  60, 150 }, // twist one way + knees compress together
  {  80,  75, 105, 100, 100, 105,  75,  80, 150 }, // twist the other way + knees extend together
  {  60, 105,  75, 120, 120,  75, 105,  60, 150 },
  {  80,  75, 105, 100, 100, 105,  75,  80, 150 },
  {  70,  90,  90, 110, 110,  90,  90,  70, 200 }, // recover -- back to standing
};

#endif
