// Arduino IDE version of led_serial.c: same commands, same 57600 baud.
// See led_serial.c for the full command list. Every command is one line ending in '\n':
//   1 / 0 / t, A<deg> / B<deg>, H9 L9 H10 L10,
//   OA <min> <max> <speed> / OB ..., GO, STOP,
//   M <a×10> <b×10> <deg/s>   glide to angles
//   T <a×10> <b×10> <ms>      timed move for time-series playback (silent)
//   ?
// Angles are limited to 45-225 deg (centre 135 +/- 90) so the table never reverses mid-move.
// Positions are reported with one decimal (A135.5); modes are RUN / MOVING / STOP.
#include <Servo.h>

// 270-degree servo: 500-2500 us = 0-270 deg, level at 135. For 180-degree servos use 180 / 544 / 2400.
const int SERVO_RANGE = 270;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_CENTRE = SERVO_RANGE / 2;
const int LIMIT_LO = SERVO_CENTRE - 90;   // 45
const int LIMIT_HI = SERVO_CENTRE + 90;   // 225

const int PINS[2] = { 9, 10 };
const int TICK_MS = 20;            // 50 Hz motion update
const int MAX_SPEED = 1000;        // deg/s
const int HOMING_MIN_SPEED = 30;   // deg/s

enum Mode { IDLE, RUNNING, MOVING };

Servo servo[2];
long pos[2] = { SERVO_CENTRE * 1000L, SERVO_CENTRE * 1000L };    // millidegrees
int dir[2] = { 1, 1 };
int lo[2] = { SERVO_CENTRE - 20, SERVO_CENTRE - 20 }, hi[2] = { SERVO_CENTRE + 20, SERVO_CENTRE + 20 }, speed[2] = { 120, 120 };
long target[2];                    // MOVING: millidegrees
long moveStep[2];                  // MOVING: millidegrees per tick
bool quiet = false;                // current move came from T: no MOVING/STOP replies
Mode mode = IDLE;
unsigned long lastTick = 0;
byte reportCount = 0;

char line[32];
byte len = 0;

void writeServo(int i) {
  long p = constrain(pos[i], LIMIT_LO * 1000L, LIMIT_HI * 1000L);
  if (servo[i].attached()) servo[i].writeMicroseconds(map(p, 0, SERVO_RANGE * 1000L, SERVO_MIN_US, SERVO_MAX_US));
}

void reportLed() { Serial.println(digitalRead(LED_BUILTIN) ? "ON" : "OFF"); }

void reportServo(int i) {
  long tenths = (pos[i] + 50) / 100;
  Serial.print(i == 0 ? 'A' : 'B');
  Serial.print(tenths / 10);
  Serial.print('.');
  Serial.println(tenths % 10);
}

void reportPin(int i) {
  Serial.print(i == 0 ? "P9=" : "P10=");
  if (servo[i].attached()) Serial.println('S');
  else Serial.println(digitalRead(PINS[i]) ? 'H' : 'L');
}

void reportOsc(int i) {
  Serial.print(i == 0 ? "OA " : "OB ");
  Serial.print(lo[i]); Serial.print(' ');
  Serial.print(hi[i]); Serial.print(' ');
  Serial.println(speed[i]);
}

void reportMode() { Serial.println(mode == RUNNING ? "RUN" : mode == MOVING ? "MOVING" : "STOP"); }

void reportAll() {
  reportLed();
  reportServo(0); reportServo(1);
  reportPin(0); reportPin(1);
  reportOsc(0); reportOsc(1);
  reportMode();
}

void attachServo(int i) {
  if (!servo[i].attached()) servo[i].attach(PINS[i], SERVO_MIN_US, SERVO_MAX_US);
  writeServo(i);
}

void tick() {
  if (mode == IDLE) return;
  bool home = true;
  for (int i = 0; i < 2; i++) {
    if (mode == RUNNING) {
      long step = (long)speed[i] * TICK_MS;   // millideg per tick
      long l = lo[i] * 1000L, h = hi[i] * 1000L;
      if (dir[i] > 0) {
        if (pos[i] > h) dir[i] = -1;
        else { pos[i] += step; if (pos[i] >= h) { pos[i] = h; dir[i] = -1; } }
      } else {
        if (pos[i] < l) dir[i] = 1;
        else { pos[i] -= step; if (pos[i] <= l) { pos[i] = l; dir[i] = 1; } }
      }
    } else {  // MOVING: glide to target
      long t = target[i], step = moveStep[i];
      if (pos[i] < t) pos[i] = min(pos[i] + step, t);
      else if (pos[i] > t) pos[i] = max(pos[i] - step, t);
      if (pos[i] != t) home = false;
    }
    writeServo(i);
  }
  if (mode == MOVING && home) {
    mode = IDLE;
    if (!quiet) { reportMode(); reportServo(0); reportServo(1); }
  }
  if (++reportCount >= 5) { reportCount = 0; reportServo(0); reportServo(1); }
}

void startMove(int a10, int b10, bool silent) {
  target[0] = constrain(a10, LIMIT_LO * 10, LIMIT_HI * 10) * 100L;   // tenths -> millidegrees
  target[1] = constrain(b10, LIMIT_LO * 10, LIMIT_HI * 10) * 100L;
  attachServo(0); attachServo(1);
  quiet = silent;
  mode = MOVING;
}

void handle(char *s) {
  if (!strcmp(s, "1")) { digitalWrite(LED_BUILTIN, HIGH); reportLed(); return; }
  if (!strcmp(s, "0")) { digitalWrite(LED_BUILTIN, LOW); reportLed(); return; }
  if (!strcmp(s, "t")) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); reportLed(); return; }
  if (!strcmp(s, "?")) { reportAll(); return; }

  if (!strcmp(s, "GO")) {
    for (int i = 0; i < 2; i++) { dir[i] = pos[i] > hi[i] * 1000L ? -1 : 1; attachServo(i); }
    mode = RUNNING;
    quiet = false;
    reportMode(); reportPin(0); reportPin(1);
    return;
  }
  if (!strcmp(s, "STOP")) {
    if (mode == RUNNING) {
      for (int i = 0; i < 2; i++) {
        target[i] = (lo[i] + hi[i]) * 500L;   // centre of range
        moveStep[i] = (long)max(speed[i], HOMING_MIN_SPEED) * TICK_MS;
      }
      mode = MOVING;
      quiet = false;
      reportMode();
    }
    return;
  }

  if (s[0] == 'M' && s[1] == ' ') {
    int a, b, v;
    if (sscanf(s + 1, "%d %d %d", &a, &b, &v) != 3) return;
    moveStep[0] = moveStep[1] = (long)constrain(v, 1, MAX_SPEED) * TICK_MS;
    startMove(a, b, false);
    reportMode();
    return;
  }

  if (s[0] == 'T' && s[1] == ' ') {
    int a, b, ms;
    if (sscanf(s + 1, "%d %d %d", &a, &b, &ms) != 3) return;
    startMove(a, b, true);
    long ticks = max(1, ms / TICK_MS);
    for (int i = 0; i < 2; i++) {
      long d = labs(target[i] - pos[i]);
      moveStep[i] = max(1L, (d + ticks - 1) / ticks);   // both axes arrive together
    }
    return;
  }

  if (s[0] == 'O' && (s[1] == 'A' || s[1] == 'B')) {
    int i = s[1] == 'A' ? 0 : 1;
    int a, b, v;
    if (sscanf(s + 2, "%d %d %d", &a, &b, &v) != 3) return;
    a = constrain(a, LIMIT_LO, LIMIT_HI); b = constrain(b, LIMIT_LO, LIMIT_HI);
    if (a > b) { int t = a; a = b; b = t; }
    lo[i] = a; hi[i] = b; speed[i] = constrain(v, 0, MAX_SPEED);
    reportOsc(i);
    return;
  }

  if (s[0] == 'A' || s[0] == 'B') {
    int i = s[0] == 'A' ? 0 : 1;
    int angle = constrain(atoi(s + 1), LIMIT_LO, LIMIT_HI);
    quiet = false;
    if (mode != IDLE) { mode = IDLE; reportMode(); }
    pos[i] = angle * 1000L;
    attachServo(i);
    reportServo(i); reportPin(i);
    return;
  }

  if (s[0] == 'H' || s[0] == 'L') {
    int pin = atoi(s + 1);
    if (pin != 9 && pin != 10) return;
    int i = pin == 9 ? 0 : 1;
    servo[i].detach();
    pinMode(pin, OUTPUT);
    digitalWrite(pin, s[0] == 'H' ? HIGH : LOW);
    reportPin(i);
    return;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  attachServo(0);
  attachServo(1);
  Serial.begin(57600);
  Serial.println("READY");
  reportAll();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) { line[len] = 0; handle(line); }
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
  unsigned long now = millis();
  if (now - lastTick >= TICK_MS) { lastTick += TICK_MS; tick(); }
}
