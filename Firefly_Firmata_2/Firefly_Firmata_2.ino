/*
  Firefly Firmata 2 — universal, runtime-configured firmware for Firefly 2 (Grasshopper).

  Version:    2.0.0  (reported as ver= in !ready/hello; the MAJOR number is the
                      wire-protocol version — see PROTOCOL.md)
  Author:     Andy Payne
  Copyright:  © 2026 Andy Payne
  License:    MIT (see the LICENSE file in this repository)
  Updated:    2026-07-16

  ONE sketch for every supported board: the host discovers what this board can do (caps?)
  and configures pins over the wire (cfg) — no sketch editing, no per-project variants.
  The wire protocol is specified in PROTOCOL.md at the root of this repository; that
  document is the source of truth and this sketch implements it. Everything is
  human-readable text at 115200 baud — open a serial monitor and type "hello".

  Protocol summary (grammar major 2):
    host → board   hello | caps? | cfg | report | set | move/moveto/stop/zero | reset
    board → host   replies: payload lines + exactly one "ok/wrn/err verb …" status line
                   events (unsolicited, "!" prefix): !ready !r !done !wrn !err
    wrn means APPLIED but not exactly as asked (value clamped, cadence floored).

  Lessons from the v1 sketch are kept deliberately: the line buffer is bounds-checked
  (overlong input is discarded and answered with err), the RX buffer is fully drained
  every loop, nothing blocks, and reporting is millis()-scheduled — an idle board is
  silent until the host subscribes.

  Steppers: motor slots (cap m<n>) drive a step/dir driver board (A4988, DRV8825,
  EasyDriver) or a 4-wire darlington stage (ULN2003 + 28BYJ-48) directly, with
  trapezoidal accel/decel ramps generated non-blockingly from loop() — the v1 sketch's
  blocking Stepper library is exactly what this replaces. No stepper library needed.
*/

// ---------------------------------------------------------------- board identity
#if defined(ARDUINO_AVR_UNO)
  #define BOARD "uno"
  #define MCU   "atmega328p"
  #define VOLTS "5.0"
#elif defined(ARDUINO_AVR_MEGA2560)
  #define BOARD "mega"
  #define MCU   "atmega2560"
  #define VOLTS "5.0"
#elif defined(ARDUINO_AVR_LEONARDO)
  #define BOARD "leonardo"
  #define MCU   "atmega32u4"
  #define VOLTS "5.0"
#elif defined(ARDUINO_AVR_YUN)
  #define BOARD "yun"
  #define MCU   "atmega32u4"
  #define VOLTS "5.0"
#elif defined(ARDUINO_SAM_DUE)
  #define BOARD "due"
  #define MCU   "sam3x8e"
  #define VOLTS "3.3"
#elif defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
  #define BOARD "unor4"
  #define MCU   "ra4m1"
  #define VOLTS "5.0"
#elif defined(ARDUINO_RASPBERRY_PI_PICO_2)
  #define BOARD "pico2"
  #define MCU   "rp2350"
  #define VOLTS "3.3"
#elif defined(ARDUINO_ARCH_RP2040)
  #define BOARD "pico"
  #define MCU   "rp2040"
  #define VOLTS "3.3"
#elif defined(ARDUINO_ARCH_ESP32)
  #define BOARD "esp32"
  #define MCU   "esp32"
  #define VOLTS "3.3"
#else
  #define BOARD "generic"
  #define MCU   "unknown"
  #if defined(__AVR__)
    #define VOLTS "5.0"
  #else
    #define VOLTS "3.3"
  #endif
#endif

// ---------------------------------------------------------------- capabilities
#define FIRMWARE_VERSION "2.0.0"   // major = wire protocol (bump ONLY on grammar breaks); minor/patch = releases
#define BAUD        115200
#define MAX_LINE    256     // spec: max line length incl. terminator, both directions (Mega-scale report lists)
#define MAX_TOKENS  14      // verb + 12 arguments + margin
#define MIN_EVERY   10      // spec: minimum report cadence in ms

#if defined(__AVR__)
  #define AIN_BITS 10
#else
  #define AIN_BITS 12       // Due / R4 / ESP32 / Pico read 12-bit once asked
#endif
#define PWM_BITS 8

// True DACs (Due only, until the R4 dac module is added).
#if defined(ARDUINO_SAM_DUE)
  #define DAC_COUNT 2
  #define DAC_BITS  12
  static const uint8_t DAC_PINS[DAC_COUNT] = { DAC0, DAC1 };
#else
  #define DAC_COUNT 0
#endif

// Servo support, gated by ARCHITECTURE — deliberately not by __has_include(<Servo.h>):
// the Arduino build system only adds a library to the include path after spotting its
// #include in the source, so an include hidden behind __has_include never gets spotted
// and the check is self-defeatingly false on every board (verified on hardware:
// caps? advertised no servo anywhere). These cores bundle or ship a Servo library;
// ESP32 does not — it stays servo-less until its own module (ESP32Servo/LEDC) lands,
// and the capability query tells the host so.
#if defined(__AVR__) || defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_RP2040)
  #include <Servo.h>
  #define HAS_SERVO   1
  // 12 matches both the AVR Servo library's single-timer capacity and, neatly, the Uno's
  // twelve servo-capable pins (d2-d13) — selecting every pin must not exhaust the pool.
  #define SERVO_SLOTS  12
#else
  #define HAS_SERVO   0
#endif

// Motor slots are a RAM/CPU budget, not a hardware property: 2 on the small AVR
// boards, 4 elsewhere. The peak step rate is bounded by loop()-scheduled stepping —
// speed= requests beyond it are clamped (wrn) rather than promised and missed.
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
  #define MOTOR_SLOTS   2
#else
  #define MOTOR_SLOTS   4
#endif
#if defined(__AVR__)
  #define MAX_STEP_RATE 2000
#else
  #define MAX_STEP_RATE 8000
#endif

// Some variants (Leonardo/Yun's 32u4 pins_arduino.h) don't define
// analogInputToDigitalPin; A0 + index is the standard layout everywhere it's missing.
#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(index) (A0 + (index))
#endif

// Cores that don't define digitalPinHasPWM get a conservative "no PWM anywhere";
// the RP2040 and ESP32 cores can PWM on any GPIO.
#ifndef digitalPinHasPWM
  #if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_ESP32)
    #define digitalPinHasPWM(p) (1)
  #else
    #define digitalPinHasPWM(p) (0)
  #endif
#endif

// ---------------------------------------------------------------- pin bookkeeping
enum PinUse : uint8_t {
  MODE_NONE = 0,
  MODE_DIN,
  MODE_DIN_PULLUP,
  MODE_DOUT,
  MODE_AIN,
  MODE_PWM,
  MODE_SERVO,
  MODE_DAC,
  MODE_MOTOR,   // claimed by a motor slot; released only by reconfiguring the motor
};

static uint8_t s_mode[NUM_DIGITAL_PINS];   // indexed by digital pin number

// A named wire target: d<n>, a<n> or dac<n>.
enum TargetKind : uint8_t { TK_NONE = 0, TK_DIGITAL, TK_ANALOG, TK_DAC };

struct Target {
  uint8_t kind;    // TargetKind
  uint8_t index;   // analog/dac index (TK_ANALOG / TK_DAC)
  uint8_t pin;     // underlying digital pin number
};

// Stepper motor slots (see the stepper section below for the motion engine).
enum MotorType : uint8_t { MT_NONE = 0, MT_STEPDIR, MT_FULL4, MT_HALF4 };

struct Motor {
  uint8_t  type;      // MotorType
  uint8_t  pinCount;  // 2 (step,dir) or 4 (IN1..IN4)
  uint8_t  pins[4];
  uint16_t speed;     // effective peak rate, steps/s
  float    accel;     // steps/s²
  float    c0;        // first-step interval, µs
  float    cmin;      // cruise interval = 1e6/speed, µs
  float    c;         // current interval, µs
  long     n;         // signed ramp counter (see the stepper section)
  long     pos;       // current position, steps (zeroed at cfg)
  long     target;
  int8_t   dir;       // +1 / -1
  uint8_t  phase;     // 4-wire coil phase cursor
  bool     moving;
  unsigned long due;  // micros() timestamp of the next step
};

static Motor s_motor[MOTOR_SLOTS];

// Pins 0/1 carry the (hardware or programming-port) serial link on the classic boards;
// they are reserved uniformly on every board for predictability.
static bool isReservedPin(uint8_t pin) {
  return pin < 2;
}

static bool isAnalogAlias(uint8_t pin) {
  for (uint8_t i = 0; i < NUM_ANALOG_INPUTS; i++)
    if (analogInputToDigitalPin(i) == pin) return true;
  return false;
}

static bool isDacPin(uint8_t pin) {
#if DAC_COUNT > 0
  for (uint8_t i = 0; i < DAC_COUNT; i++)
    if (DAC_PINS[i] == pin) return true;
#else
  (void) pin;
#endif
  return false;
}

// ---------------------------------------------------------------- servo pool
#if HAS_SERVO
static Servo   s_servo[SERVO_SLOTS];
static uint8_t s_servoPin[SERVO_SLOTS];   // 255 = slot free

static int8_t servoSlotFor(uint8_t pin) {
  for (uint8_t i = 0; i < SERVO_SLOTS; i++)
    if (s_servoPin[i] == pin) return (int8_t) i;
  return -1;
}

static bool attachServo(uint8_t pin) {
  if (servoSlotFor(pin) >= 0) return true;
  for (uint8_t i = 0; i < SERVO_SLOTS; i++) {
    if (s_servoPin[i] == 255) {
      s_servoPin[i] = pin;
      s_servo[i].attach(pin);
      return true;
    }
  }
  return false;   // pool exhausted
}

static void detachServo(uint8_t pin) {
  int8_t slot = servoSlotFor(pin);
  if (slot >= 0) {
    s_servo[slot].detach();
    s_servoPin[slot] = 255;
  }
}
#endif

// ---------------------------------------------------------------- stepper motors
// Non-blocking trapezoidal motion, stepped from loop() on a micros() schedule. The ramp
// follows D. Austin, "Generate stepper-motor speed profiles in real time" (Embedded
// Systems Programming, 2005): the first step interval is c0 = 1e6 * sqrt(2/accel) µs and
// each accelerating step shortens it by c -= 2c/(4n+1). The signed ramp counter n mirrors
// that series: n > 0 accelerating (n ≈ braking distance in steps), n < 0 decelerating
// (|n| steps to standstill), and the braking need is recomputed every step from the
// actual speed — so retargeting mid-move just works (overshoot, brake, come back).
// (MotorType/Motor are declared with the other types in the pin-bookkeeping section —
// the Arduino builder inserts auto-generated prototypes at the FIRST function definition
// in the sketch, so every type a function signature mentions must be declared above it.)

// "m1" → slot 0; anything unknown → -1.
static int8_t parseMotor(const char* name) {
  if (name == NULL || name[0] != 'm') return -1;
  uint32_t index;
  if (!parseUInt(name + 1, &index) || index < 1 || index > MOTOR_SLOTS) return -1;
  return (int8_t)(index - 1);
}

// Which motor slot holds a pin, or -1. (The per-pin MODE_MOTOR marker says "a motor";
// this says which one, for the "claimed m<n>" error.)
static int8_t motorHolding(uint8_t pin) {
  for (uint8_t m = 0; m < MOTOR_SLOTS; m++) {
    if (s_motor[m].type == MT_NONE) continue;
    for (uint8_t i = 0; i < s_motor[m].pinCount; i++)
      if (s_motor[m].pins[i] == pin) return (int8_t) m;
  }
  return -1;
}

static void motorEmitDone(uint8_t index) {
  Serial.print(F("!done m"));
  Serial.print(index + 1);
  Serial.print(F(" pos="));
  Serial.print(s_motor[index].pos);
  Serial.print('\n');
}

// Coil patterns, IN1..IN4 msb-first. Full-step keeps two coils on (torque); half-step
// interleaves single-coil states (smoother, twice the steps per revolution).
static void motorWritePhase(Motor& motor) {
  static const uint8_t FULL4[4] = { 0b1100, 0b0110, 0b0011, 0b1001 };
  static const uint8_t HALF4[8] = { 0b1000, 0b1100, 0b0100, 0b0110, 0b0010, 0b0011, 0b0001, 0b1001 };
  uint8_t pattern = (motor.type == MT_FULL4) ? FULL4[motor.phase & 3] : HALF4[motor.phase & 7];
  for (uint8_t i = 0; i < 4; i++)
    digitalWrite(motor.pins[i], (pattern >> (3 - i)) & 1);
}

static void motorStepOnce(Motor& motor) {
  if (motor.type == MT_STEPDIR) {
    digitalWrite(motor.pins[1], motor.dir > 0 ? HIGH : LOW);
    digitalWrite(motor.pins[0], HIGH);   // A4988/A3967 latch on the rising edge (≥1 µs high)
    delayMicroseconds(2);
    digitalWrite(motor.pins[0], LOW);
  }
  else {
    motor.phase = (uint8_t)(motor.phase + motor.dir);   // wraps; masked in motorWritePhase
    motorWritePhase(motor);
  }
  motor.pos += motor.dir;
}

// Halts and unclaims a slot: coils off, pins floated, position forgotten. Deliberately
// NO !done — this only happens on cfg/reset, which the host initiated and knows about.
static void releaseMotor(uint8_t index) {
  Motor& motor = s_motor[index];
  if (motor.type == MT_NONE) return;
  for (uint8_t i = 0; i < motor.pinCount; i++) {
    digitalWrite(motor.pins[i], LOW);
    pinMode(motor.pins[i], INPUT);
    s_mode[motor.pins[i]] = MODE_NONE;
  }
  motor.type = MT_NONE;
  motor.moving = false;
  motor.pos = 0;
  motor.target = 0;
}

// Begins motion toward motor.target from standstill. (While already moving, a retarget
// needs nothing: the per-step ramp logic below re-aims itself.)
static void motorStart(Motor& motor) {
  if (motor.moving || motor.target == motor.pos) return;
  motor.dir = (motor.target > motor.pos) ? 1 : -1;
  motor.n = 0;
  motor.c = motor.c0;
  motor.due = micros();
  motor.moving = true;
}

// One scheduler pass for one motor: take the due step, then plan the next interval.
static void motorTick(uint8_t index) {
  Motor& motor = s_motor[index];
  if (motor.type == MT_NONE || !motor.moving) return;

  unsigned long now = micros();
  if ((long)(now - motor.due) < 0) return;

  motorStepOnce(motor);

  long distance = motor.target - motor.pos;
  float velocity = 1000000.0f / motor.c;                              // steps/s after this step
  long braking = (long)((velocity * velocity) / (2.0f * motor.accel)); // steps to standstill

  if (distance == 0 && braking <= 1) {
    motor.moving = false;
    motor.n = 0;
    motorEmitDone(index);
    return;
  }

  // Plan: flip the ramp counter negative when we must brake — wrong direction, or the
  // target is inside braking distance — and positive again when clear road returns.
  if (distance > 0) {
    if (motor.n > 0 && (braking >= distance || motor.dir < 0)) motor.n = -braking;
    else if (motor.n < 0 && braking < distance && motor.dir > 0) motor.n = -motor.n;
  }
  else if (distance < 0) {
    if (motor.n > 0 && (braking >= -distance || motor.dir > 0)) motor.n = -braking;
    else if (motor.n < 0 && braking < -distance && motor.dir < 0) motor.n = -motor.n;
  }
  else {
    motor.n = -braking;   // parked on the target but still fast: brake through, come back
  }

  if (motor.n == 0) {
    // Standstill mid-plan (a braked reversal): relaunch toward the target.
    motor.c = motor.c0;
    motor.dir = (distance > 0) ? 1 : -1;
  }
  else {
    motor.c = motor.c - 2.0f * motor.c / (4.0f * motor.n + 1);   // Austin's series; n<0 lengthens
    if (motor.c < motor.cmin) motor.c = motor.cmin;
  }
  motor.n++;

  motor.due = now + (unsigned long) motor.c;
}

// ---------------------------------------------------------------- report subscription
#define MAX_REPORT 16

struct ReportEntry {
  uint8_t kind;    // TK_ANALOG (a<n>) or TK_DIGITAL (d<n>)
  uint8_t index;   // analog index
  uint8_t pin;     // digital pin
  int16_t last;    // last SENT value (delta mode)
};

static ReportEntry s_report[MAX_REPORT];
static uint8_t        s_reportCount = 0;
static uint16_t       s_reportEvery = 0;
static uint16_t       s_reportDelta = 0;     // 0 = send every sample
static bool           s_reportPrimed = false; // first sample always sent
static unsigned long  s_reportLast = 0;

// ---------------------------------------------------------------- line input
static char    s_line[MAX_LINE];
static uint8_t s_lineLen = 0;
static bool    s_lineOverflow = false;

// ---------------------------------------------------------------- small helpers
static bool parseUInt(const char* s, uint32_t* out) {
  if (s == NULL || *s == 0) return false;
  uint32_t value = 0;
  for (; *s; s++) {
    if (*s < '0' || *s > '9') return false;
    value = value * 10 + (uint32_t)(*s - '0');
    if (value > 1000000UL) return false;   // nothing on this wire is that large
  }
  *out = value;
  return true;
}

static bool parseInt(const char* s, long* out) {
  if (s == NULL || *s == 0) return false;
  bool negative = false;
  if (*s == '-') { negative = true; s++; }
  uint32_t magnitude;
  if (!parseUInt(s, &magnitude)) return false;
  *out = negative ? -(long) magnitude : (long) magnitude;
  return true;
}

// In-place space tokenizer (single spaces per spec, but runs are tolerated).
static uint8_t tokenize(char* line, char** argv, uint8_t maxTokens) {
  uint8_t argc = 0;
  char* p = line;
  while (*p && argc < maxTokens) {
    while (*p == ' ') p++;
    if (*p == 0) break;
    argv[argc++] = p;
    while (*p && *p != ' ') p++;
    if (*p) *p++ = 0;
  }
  return argc;
}

// Resolves "d13" / "a0" / "dac0" to a target we advertise in caps?. Names we do not
// advertise (d0/d1, an analog pin's d-alias, out-of-range indices) resolve to TK_NONE.
static Target parseTarget(const char* name) {
  Target target = { TK_NONE, 0, 0 };
  if (name == NULL) return target;

  uint32_t index;
  if (name[0] == 'd' && name[1] == 'a' && name[2] == 'c') {
#if DAC_COUNT > 0
    if (parseUInt(name + 3, &index) && index < DAC_COUNT) {
      target.kind = TK_DAC;
      target.index = (uint8_t) index;
      target.pin = DAC_PINS[index];
    }
#endif
  }
  else if (name[0] == 'a') {
    if (parseUInt(name + 1, &index) && index < NUM_ANALOG_INPUTS) {
      target.kind = TK_ANALOG;
      target.index = (uint8_t) index;
      target.pin = analogInputToDigitalPin(index);
    }
  }
  else if (name[0] == 'd') {
    if (parseUInt(name + 1, &index) && index < NUM_DIGITAL_PINS
        && !isReservedPin((uint8_t) index)
        && !isAnalogAlias((uint8_t) index)
        && !isDacPin((uint8_t) index)) {
      target.kind = TK_DIGITAL;
      target.pin = (uint8_t) index;
    }
  }
  return target;
}

static void printTargetName(const Target& target) {
  switch (target.kind) {
    case TK_ANALOG:  Serial.print('a');       Serial.print(target.index); break;
    case TK_DAC:     Serial.print(F("dac"));  Serial.print(target.index); break;
    default:         Serial.print('d');       Serial.print(target.pin);   break;
  }
}

static PinUse parseMode(const char* name) {
  if (name == NULL) return MODE_NONE;
  if (strcmp(name, "din") == 0)   return MODE_DIN;
  if (strcmp(name, "dout") == 0)  return MODE_DOUT;
  if (strcmp(name, "ain") == 0)   return MODE_AIN;
  if (strcmp(name, "pwm") == 0)   return MODE_PWM;
  if (strcmp(name, "servo") == 0) return MODE_SERVO;
  if (strcmp(name, "dac") == 0)   return MODE_DAC;
  return MODE_NONE;
}

// The single support predicate — caps? emission and cfg validation both use it, so the
// board can never advertise a mode it would then refuse.
static bool supports(const Target& target, PinUse mode) {
  switch (target.kind) {
    case TK_ANALOG:
      return mode == MODE_AIN || mode == MODE_DIN || mode == MODE_DIN_PULLUP || mode == MODE_DOUT;
    case TK_DIGITAL:
      if (mode == MODE_DIN || mode == MODE_DIN_PULLUP || mode == MODE_DOUT) return true;
      if (mode == MODE_PWM) return digitalPinHasPWM(target.pin);
#if HAS_SERVO
      if (mode == MODE_SERVO) return true;
#endif
      return false;
    case TK_DAC:
      return mode == MODE_DAC;
    default:
      return false;
  }
}

static void tearDownPin(uint8_t pin) {
#if HAS_SERVO
  if (s_mode[pin] == MODE_SERVO) detachServo(pin);
#endif
  if (s_mode[pin] != MODE_NONE) pinMode(pin, INPUT);   // safe floating default
  s_mode[pin] = MODE_NONE;
}

static int maxValueFor(PinUse mode) {
  switch (mode) {
    case MODE_DOUT:  return 1;
    case MODE_PWM:   return (1 << PWM_BITS) - 1;
    case MODE_SERVO: return 180;
#if DAC_COUNT > 0
    case MODE_DAC:   return (1 << DAC_BITS) - 1;
#endif
    default:         return 0;
  }
}

static void writeOutput(const Target& target, int value) {
  switch (s_mode[target.pin]) {
    case MODE_DOUT:
      digitalWrite(target.pin, value ? HIGH : LOW);
      break;
    case MODE_PWM:
#if defined(ARDUINO_SAM_DUE)
      analogWriteResolution(PWM_BITS);   // global on the Due; pin it per write
#endif
      analogWrite(target.pin, value);
      break;
#if HAS_SERVO
    case MODE_SERVO: {
      int8_t slot = servoSlotFor(target.pin);
      if (slot >= 0) s_servo[slot].write(value);
      break;
    }
#endif
#if DAC_COUNT > 0
    case MODE_DAC:
      analogWriteResolution(DAC_BITS);
      analogWrite(target.pin, value);
      break;
#endif
    default:
      break;
  }
}

// ---------------------------------------------------------------- replies & events
static void printIdentity() {
  Serial.print(F("fw=firefly2 ver="));
  Serial.print(F(FIRMWARE_VERSION));
  Serial.print(F(" board="));
  Serial.print(F(BOARD));
}

static void sendReady() {
  Serial.print(F("!ready "));
  printIdentity();
  Serial.print('\n');
}

// ---------------------------------------------------------------- command handlers
static void handleHello() {
  Serial.print(F("ok hello "));
  printIdentity();
  Serial.print(F(" mcu="));
  Serial.print(F(MCU));
  Serial.print(F(" volts="));
  Serial.print(F(VOLTS));
  Serial.print('\n');
}

static void handleCaps() {
  uint16_t count = 0;

  for (uint8_t i = 0; i < NUM_ANALOG_INPUTS; i++) {
    Serial.print(F("cap a"));
    Serial.print(i);
    Serial.print(F(" modes=ain,din,dout ain="));
    Serial.print(AIN_BITS);
    Serial.print('\n');
    count++;
  }

  for (uint8_t pin = 2; pin < NUM_DIGITAL_PINS; pin++) {
    if (isAnalogAlias(pin) || isDacPin(pin)) continue;
    Serial.print(F("cap d"));
    Serial.print(pin);
    Serial.print(F(" modes=din,dout"));
    if (digitalPinHasPWM(pin)) Serial.print(F(",pwm"));
#if HAS_SERVO
    Serial.print(F(",servo"));
#endif
    if (digitalPinHasPWM(pin)) {
      Serial.print(F(" pwm="));
      Serial.print(PWM_BITS);
    }
    Serial.print('\n');
    count++;
  }

#if DAC_COUNT > 0
  for (uint8_t i = 0; i < DAC_COUNT; i++) {
    Serial.print(F("cap dac"));
    Serial.print(i);
    Serial.print(F(" modes=dac dac="));
    Serial.print(DAC_BITS);
    Serial.print('\n');
    count++;
  }
#endif

  for (uint8_t m = 0; m < MOTOR_SLOTS; m++) {
    Serial.print(F("cap m"));
    Serial.print(m + 1);
    Serial.print(F(" modes=stepper\n"));
    count++;
  }

  Serial.print(F("ok caps "));
  Serial.print(count);
  Serial.print('\n');
}

static void handleCfg(uint8_t argc, char** argv) {
  if (argc < 3) {
    Serial.print(F("err cfg malformed\n"));
    return;
  }

  // Motor slots (m<n>) are firmware resources with their own grammar.
  if (argv[1][0] == 'm' && argv[1][1] >= '0' && argv[1][1] <= '9') {
    handleCfgStepper(argc, argv);
    return;
  }

  Target target = parseTarget(argv[1]);
  if (target.kind == TK_NONE) {
    Serial.print(F("err cfg "));
    Serial.print(argv[1]);
    Serial.print(F(" unknown\n"));
    return;
  }

  // A pin held by a motor is only released by reconfiguring the motor — spec:
  // "err cfg d2 dout claimed m1".
  int8_t holder = motorHolding(target.pin);
  if (holder >= 0) {
    Serial.print(F("err cfg "));
    printTargetName(target);
    Serial.print(' ');
    Serial.print(argv[2]);
    Serial.print(F(" claimed m"));
    Serial.print(holder + 1);
    Serial.print('\n');
    return;
  }

  PinUse mode = parseMode(argv[2]);
  bool pullup = (argc > 3 && strcmp(argv[3], "pullup") == 0);
  if (mode == MODE_DIN && pullup) mode = MODE_DIN_PULLUP;

  if (mode == MODE_NONE || !supports(target, mode)) {
    Serial.print(F("err cfg "));
    printTargetName(target);
    Serial.print(' ');
    Serial.print(argv[2]);
    Serial.print(F(" unsupported\n"));
    return;
  }

  // Reconfiguring is always allowed: the last cfg wins.
  tearDownPin(target.pin);

  switch (mode) {
    case MODE_DIN:        pinMode(target.pin, INPUT);        break;
    case MODE_DIN_PULLUP: pinMode(target.pin, INPUT_PULLUP); break;
    case MODE_DOUT:       pinMode(target.pin, OUTPUT);       break;
    case MODE_AIN:        /* analogRead needs no pinMode */  break;
    case MODE_PWM:        pinMode(target.pin, OUTPUT);       break;
#if HAS_SERVO
    case MODE_SERVO:
      if (!attachServo(target.pin)) {
        Serial.print(F("err cfg "));
        printTargetName(target);
        Serial.print(F(" servo noslots\n"));
        return;
      }
      break;
#endif
    case MODE_DAC:        /* configured per write */         break;
    default: break;
  }

  s_mode[target.pin] = (uint8_t) mode;

  Serial.print(F("ok cfg "));
  printTargetName(target);
  Serial.print(' ');
  Serial.print(argv[2]);
  if (mode == MODE_DIN_PULLUP) Serial.print(F(" pullup"));
  Serial.print('\n');
}

// cfg m<n> stepper [type=stepdir|full4|half4] step=<pin> dir=<pin> | pins=<p,p,p,p>
//                  [speed=<steps/s>] [accel=<steps/s²>]
// Validates everything before touching state (a cfg line is atomic), then releases the
// slot's previous pins, claims the new ones and parks the motor at position 0.
static void handleCfgStepper(uint8_t argc, char** argv) {
  int8_t index = parseMotor(argv[1]);
  if (index < 0) {
    Serial.print(F("err cfg "));
    Serial.print(argv[1]);
    Serial.print(F(" unknown\n"));
    return;
  }
  if (strcmp(argv[2], "stepper") != 0) {
    Serial.print(F("err cfg m"));
    Serial.print(index + 1);
    Serial.print(' ');
    Serial.print(argv[2]);
    Serial.print(F(" unsupported\n"));
    return;
  }

  const char* typeName = "stepdir";
  const char* stepName = NULL;
  const char* dirName = NULL;
  char* pinList = NULL;
  uint32_t speed = 1000;
  uint32_t accel = 1000;

  for (uint8_t i = 3; i < argc; i++) {
    char* equals = strchr(argv[i], '=');
    if (equals == NULL) {
      Serial.print(F("err cfg m"));
      Serial.print(index + 1);
      Serial.print(F(" stepper malformed\n"));
      return;
    }
    *equals = 0;
    char* value = equals + 1;
    if      (strcmp(argv[i], "type") == 0)  typeName = value;
    else if (strcmp(argv[i], "step") == 0)  stepName = value;
    else if (strcmp(argv[i], "dir") == 0)   dirName = value;
    else if (strcmp(argv[i], "pins") == 0)  pinList = value;
    else if (strcmp(argv[i], "speed") == 0 || strcmp(argv[i], "accel") == 0) {
      uint32_t parsed;
      if (!parseUInt(value, &parsed)) {
        Serial.print(F("err cfg m"));
        Serial.print(index + 1);
        Serial.print(F(" stepper malformed\n"));
        return;
      }
      if (argv[i][0] == 's') speed = parsed; else accel = parsed;
    }
    // Unknown keys are ignored (forward compatibility), like unknown lines.
  }

  uint8_t type;
  if      (strcmp(typeName, "stepdir") == 0) type = MT_STEPDIR;
  else if (strcmp(typeName, "full4") == 0)   type = MT_FULL4;
  else if (strcmp(typeName, "half4") == 0)   type = MT_HALF4;
  else {
    Serial.print(F("err cfg m"));
    Serial.print(index + 1);
    Serial.print(F(" stepper unsupported\n"));
    return;
  }

  // Resolve the pins: step,dir for a driver board; IN1..IN4 for a darlington stage.
  Target resolved[4];
  uint8_t pinCount = 0;
  if (type == MT_STEPDIR) {
    resolved[pinCount] = parseTarget(stepName);
    if (resolved[pinCount].kind != TK_NONE && resolved[pinCount].kind != TK_DAC) pinCount++;
    resolved[pinCount] = parseTarget(dirName);
    if (resolved[pinCount].kind != TK_NONE && resolved[pinCount].kind != TK_DAC) pinCount++;
    if (pinCount != 2) {
      Serial.print(F("err cfg m"));
      Serial.print(index + 1);
      Serial.print(F(" stepper malformed\n"));
      return;
    }
  }
  else {
    char* cursor = pinList;
    while (cursor != NULL && *cursor) {
      char* name = cursor;
      while (*cursor && *cursor != ',') cursor++;
      if (*cursor == ',') *cursor++ = 0;
      if (pinCount >= 4) { pinCount = 5; break; }   // too many
      Target target = parseTarget(name);
      if (target.kind == TK_NONE || target.kind == TK_DAC) { pinCount = 5; break; }
      resolved[pinCount++] = target;
    }
    if (pinCount != 4) {
      Serial.print(F("err cfg m"));
      Serial.print(index + 1);
      Serial.print(F(" stepper malformed\n"));
      return;
    }
  }

  // No duplicates, and no pin held by ANOTHER motor (this slot's own pins are about to
  // be released, so they don't count).
  for (uint8_t i = 0; i < pinCount; i++) {
    for (uint8_t j = i + 1; j < pinCount; j++) {
      if (resolved[i].pin == resolved[j].pin) {
        Serial.print(F("err cfg m"));
        Serial.print(index + 1);
        Serial.print(F(" stepper malformed\n"));
        return;
      }
    }
    int8_t holder = motorHolding(resolved[i].pin);
    if (holder >= 0 && holder != index) {
      Serial.print(F("err cfg m"));
      Serial.print(index + 1);
      Serial.print(F(" stepper claimed m"));
      Serial.print(holder + 1);
      Serial.print('\n');
      return;
    }
  }

  bool clamped = false;
  if (speed < 1)             { speed = 1;             clamped = true; }
  if (speed > MAX_STEP_RATE) { speed = MAX_STEP_RATE; clamped = true; }
  if (accel < 1)             { accel = 1;             clamped = true; }

  // Same type and pins: a RETUNE, not a reconfigure. Position, target, coil phase and
  // any motion in flight are preserved — a speed slider dragged mid-move slows or
  // quickens the motion, it doesn't kill it (and an idle motor keeps its position, so
  // absolute moveto targets stay meaningful across tuning).
  Motor& motor = s_motor[index];
  bool retune = motor.type == type && motor.pinCount == pinCount;
  if (retune) {
    for (uint8_t i = 0; i < pinCount; i++)
      if (motor.pins[i] != resolved[i].pin) { retune = false; break; }
  }

  if (!retune) {
    // Validated — apply fresh. Last cfg wins over regular pin modes, so tear those down.
    releaseMotor((uint8_t) index);
    for (uint8_t i = 0; i < pinCount; i++) {
      tearDownPin(resolved[i].pin);
      pinMode(resolved[i].pin, OUTPUT);
      digitalWrite(resolved[i].pin, LOW);
      s_mode[resolved[i].pin] = MODE_MOTOR;
      motor.pins[i] = resolved[i].pin;
    }
    motor.type = type;
    motor.pinCount = pinCount;
    motor.pos = 0;
    motor.target = 0;
    motor.n = 0;
    motor.dir = 1;
    motor.phase = 0;
    motor.moving = false;
    if (type != MT_STEPDIR) motorWritePhase(motor);   // energise coils (holding torque)
  }

  motor.speed = (uint16_t) speed;
  motor.accel = (float) accel;
  motor.cmin = 1000000.0f / (float) speed;
  motor.c0 = 1000000.0f * sqrt(2.0f / motor.accel);
  if (motor.c0 < motor.cmin) motor.c0 = motor.cmin;

  if (retune && motor.moving) {
    // Continue the flight under the new limits: cap the current cadence at the new peak
    // (an instant slow-down never skips steps) and rebuild the ramp counter from the
    // actual speed — the per-step planner takes it from there.
    if (motor.c < motor.cmin) motor.c = motor.cmin;
    float velocity = 1000000.0f / motor.c;
    motor.n = (long)((velocity * velocity) / (2.0f * motor.accel));
    if (motor.n < 1) motor.n = 1;
  }

  Serial.print(clamped ? F("wrn cfg m") : F("ok cfg m"));
  Serial.print(index + 1);
  Serial.print(F(" stepper"));
  if (type == MT_STEPDIR) {
    Serial.print(F(" step="));
    printTargetName(resolved[0]);
    Serial.print(F(" dir="));
    printTargetName(resolved[1]);
  }
  else {
    Serial.print(F(" type="));
    Serial.print(typeName);
    Serial.print(F(" pins="));
    for (uint8_t i = 0; i < pinCount; i++) {
      if (i > 0) Serial.print(',');
      printTargetName(resolved[i]);
    }
  }
  Serial.print(F(" speed="));
  Serial.print(speed);
  Serial.print(F(" accel="));
  Serial.print(accel);
  if (clamped) Serial.print(F(" clamped"));
  Serial.print('\n');
}

static void stopReporting() {
  s_reportCount = 0;
  s_reportEvery = 0;
  s_reportDelta = 0;
  s_reportPrimed = false;
}

static void handleReport(uint8_t argc, char** argv) {
  if (argc >= 2 && strcmp(argv[1], "none") == 0) {
    stopReporting();
    Serial.print(F("ok report none\n"));
    return;
  }

  if (argc < 4 || strcmp(argv[2], "every") != 0) {
    Serial.print(F("err report malformed\n"));
    return;
  }

  uint32_t every;
  if (!parseUInt(argv[3], &every)) {
    Serial.print(F("err report malformed\n"));
    return;
  }

  uint32_t delta = 0;
  bool hasDelta = false;
  if (argc >= 6 && strcmp(argv[4], "delta") == 0) {
    if (!parseUInt(argv[5], &delta)) {
      Serial.print(F("err report malformed\n"));
      return;
    }
    hasDelta = true;
  }

  // Parse the comma list into a staging area; the live subscription is only replaced
  // when the whole command validates.
  ReportEntry staged[MAX_REPORT];
  uint8_t stagedCount = 0;

  char* cursor = argv[1];
  while (*cursor) {
    char* name = cursor;
    while (*cursor && *cursor != ',') cursor++;
    if (*cursor == ',') *cursor++ = 0;

    if (stagedCount >= MAX_REPORT) {
      Serial.print(F("err report toomany\n"));
      return;
    }

    Target target = parseTarget(name);
    bool readable =
      (target.kind == TK_ANALOG && s_mode[target.pin] == MODE_AIN) ||
      (target.kind != TK_NONE && (s_mode[target.pin] == MODE_DIN || s_mode[target.pin] == MODE_DIN_PULLUP));
    if (!readable) {
      Serial.print(F("err report "));
      Serial.print(name);
      Serial.print(F(" unconfigured\n"));
      return;
    }

    staged[stagedCount].kind = (s_mode[target.pin] == MODE_AIN) ? TK_ANALOG : TK_DIGITAL;
    staged[stagedCount].index = target.index;
    staged[stagedCount].pin = target.pin;
    staged[stagedCount].last = 0;
    stagedCount++;
  }

  if (stagedCount == 0) {
    Serial.print(F("err report malformed\n"));
    return;
  }

  bool floored = every < MIN_EVERY;
  if (floored) every = MIN_EVERY;

  memcpy(s_report, staged, sizeof(staged[0]) * stagedCount);
  s_reportCount = stagedCount;
  s_reportEvery = (uint16_t) every;
  s_reportDelta = (uint16_t) delta;
  s_reportPrimed = false;              // first sample always goes out
  s_reportLast = millis();

  Serial.print(floored ? F("wrn report ") : F("ok report "));
  for (uint8_t i = 0; i < s_reportCount; i++) {
    if (i > 0) Serial.print(',');
    if (s_report[i].kind == TK_ANALOG) { Serial.print('a'); Serial.print(s_report[i].index); }
    else                               { Serial.print('d'); Serial.print(s_report[i].pin); }
  }
  Serial.print(F(" every "));
  Serial.print(every);
  if (hasDelta) {
    Serial.print(F(" delta "));
    Serial.print(delta);
  }
  if (floored) Serial.print(F(" floored"));
  Serial.print('\n');
}

static void handleSet(uint8_t argc, char** argv) {
  if (argc < 2) {
    Serial.print(F("err set malformed\n"));
    return;
  }

  // Pass 1: validate every pair — a set line is atomic, nothing applies on err.
  Target targets[MAX_TOKENS];
  long values[MAX_TOKENS];
  bool clamped = false;
  uint8_t pairs = 0;

  for (uint8_t i = 1; i < argc; i++) {
    char* equals = strchr(argv[i], '=');
    if (equals == NULL) {
      Serial.print(F("err set malformed\n"));
      return;
    }
    *equals = 0;

    Target target = parseTarget(argv[i]);
    long value;
    if (target.kind == TK_NONE || !parseInt(equals + 1, &value)) {
      Serial.print(F("err set "));
      Serial.print(argv[i]);
      Serial.print(F(" malformed\n"));
      return;
    }

    PinUse mode = (PinUse) s_mode[target.pin];
    if (mode != MODE_DOUT && mode != MODE_PWM && mode != MODE_SERVO && mode != MODE_DAC) {
      Serial.print(F("err set "));
      printTargetName(target);
      Serial.print(F(" unconfigured\n"));
      return;
    }

    long max = maxValueFor(mode);
    if (value < 0)   { value = 0;   clamped = true; }
    if (value > max) { value = max; clamped = true; }

    targets[pairs] = target;
    values[pairs] = value;
    pairs++;
  }

  // Pass 2: apply and echo the EFFECTIVE values (wrn when anything was clamped).
  Serial.print(clamped ? F("wrn set") : F("ok set"));
  for (uint8_t i = 0; i < pairs; i++) {
    writeOutput(targets[i], (int) values[i]);
    Serial.print(' ');
    printTargetName(targets[i]);
    Serial.print('=');
    Serial.print(values[i]);
  }
  if (clamped) Serial.print(F(" clamped"));
  Serial.print('\n');
}

static void handleReset() {
  stopReporting();
  // Motors first (silently — the host initiated this and knows): coils off, slots freed.
  for (uint8_t m = 0; m < MOTOR_SLOTS; m++)
    releaseMotor(m);
  for (uint8_t pin = 0; pin < NUM_DIGITAL_PINS; pin++)
    tearDownPin(pin);
  Serial.print(F("ok reset\n"));
}

// move <motor> <steps> (relative) / moveto <motor> <pos> (absolute). The ok acknowledges
// ACCEPTANCE; completion arrives later as !done. Retargeting mid-move needs no special
// case — the ramp re-plans itself every step.
static void handleMove(bool absolute, uint8_t argc, char** argv) {
  const char* verb = absolute ? "moveto" : "move";

  long value;
  int8_t index = argc >= 3 ? parseMotor(argv[1]) : -1;
  if (index < 0 || !parseInt(argv[2], &value)) {
    Serial.print(F("err "));
    Serial.print(verb);
    Serial.print(F(" malformed\n"));
    return;
  }

  Motor& motor = s_motor[index];
  if (motor.type == MT_NONE) {
    Serial.print(F("err "));
    Serial.print(verb);
    Serial.print(F(" m"));
    Serial.print(index + 1);
    Serial.print(F(" unconfigured\n"));
    return;
  }

  motor.target = absolute ? value : motor.pos + value;

  Serial.print(F("ok "));
  Serial.print(verb);
  Serial.print(F(" m"));
  Serial.print(index + 1);
  Serial.print(' ');
  Serial.print(value);
  Serial.print('\n');

  if (!motor.moving) {
    if (motor.target == motor.pos) motorEmitDone((uint8_t) index);   // zero-length motion
    else motorStart(motor);
  }
}

// stop <motor> — decelerate to a halt; !done reports where the motor came to rest.
static void handleStop(uint8_t argc, char** argv) {
  int8_t index = argc >= 2 ? parseMotor(argv[1]) : -1;
  if (index < 0) {
    Serial.print(F("err stop malformed\n"));
    return;
  }

  Motor& motor = s_motor[index];
  if (motor.type == MT_NONE) {
    Serial.print(F("err stop m"));
    Serial.print(index + 1);
    Serial.print(F(" unconfigured\n"));
    return;
  }

  Serial.print(F("ok stop m"));
  Serial.print(index + 1);
  Serial.print('\n');

  if (motor.moving) {
    // Aim at the nearest reachable standstill; the ramp brakes there and emits !done.
    float velocity = 1000000.0f / motor.c;
    long braking = (long)((velocity * velocity) / (2.0f * motor.accel));
    motor.target = motor.pos + motor.dir * braking;
    if (motor.target == motor.pos) motor.target = motor.pos + motor.dir;   // at least finish this step
  }
  else {
    motorEmitDone((uint8_t) index);   // already at rest
  }
}

// zero <motor> — homing: declare the current position to be 0. Only meaningful at rest
// (the calibration flow is "come to rest on the reference point, then zero"); a moving
// motor is refused, since it is ambiguous where its in-flight motion should now end.
static void handleZero(uint8_t argc, char** argv) {
  int8_t index = argc >= 2 ? parseMotor(argv[1]) : -1;
  if (index < 0) {
    Serial.print(F("err zero malformed\n"));
    return;
  }

  Motor& motor = s_motor[index];
  if (motor.type == MT_NONE) {
    Serial.print(F("err zero m"));
    Serial.print(index + 1);
    Serial.print(F(" unconfigured\n"));
    return;
  }
  if (motor.moving) {
    Serial.print(F("err zero m"));
    Serial.print(index + 1);
    Serial.print(F(" moving\n"));
    return;
  }

  motor.pos = 0;
  motor.target = 0;
  Serial.print(F("ok zero m"));
  Serial.print(index + 1);
  Serial.print('\n');
}

// ---------------------------------------------------------------- line dispatch
static void handleLine(char* line) {
  char* argv[MAX_TOKENS];
  uint8_t argc = tokenize(line, argv, MAX_TOKENS);
  if (argc == 0) return;   // blank line: ignore

  const char* verb = argv[0];
  if      (strcmp(verb, "hello") == 0)  handleHello();
  else if (strcmp(verb, "caps?") == 0)  handleCaps();
  else if (strcmp(verb, "cfg") == 0)    handleCfg(argc, argv);
  else if (strcmp(verb, "report") == 0) handleReport(argc, argv);
  else if (strcmp(verb, "set") == 0)    handleSet(argc, argv);
  else if (strcmp(verb, "reset") == 0)  handleReset();
  else if (strcmp(verb, "move") == 0)   handleMove(false, argc, argv);
  else if (strcmp(verb, "moveto") == 0) handleMove(true, argc, argv);
  else if (strcmp(verb, "stop") == 0)   handleStop(argc, argv);
  else if (strcmp(verb, "zero") == 0)   handleZero(argc, argv);
  // Unrecognised lines are ignored per spec — never fatal.
}

static void readSerial() {
  // Drain the whole RX buffer every loop (v1 lesson: partial drains corrupt lines at
  // high input rates).
  while (Serial.available() > 0) {
    char c = (char) Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      s_line[s_lineLen] = 0;
      if (s_lineOverflow) Serial.print(F("err line overflow\n"));
      else handleLine(s_line);
      s_lineLen = 0;
      s_lineOverflow = false;
    }
    else if (s_lineLen < MAX_LINE - 1) {
      s_line[s_lineLen++] = c;
    }
    else {
      s_lineOverflow = true;   // discard until the newline, then reply err
    }
  }
}

// ---------------------------------------------------------------- reporting
static void reportTick() {
  if (s_reportCount == 0) return;

  unsigned long now = millis();
  if (now - s_reportLast < s_reportEvery) return;
  s_reportLast = now;

  int16_t sample[MAX_REPORT];
  bool changed = !s_reportPrimed;   // the first sample after (re)subscribing always goes out

  for (uint8_t i = 0; i < s_reportCount; i++) {
    if (s_report[i].kind == TK_ANALOG)
      sample[i] = (int16_t) analogRead(analogInputToDigitalPin(s_report[i].index));
    else
      sample[i] = (int16_t) digitalRead(s_report[i].pin);

    if (s_reportDelta > 0 && !changed) {
      int16_t difference = sample[i] - s_report[i].last;
      if (difference < 0) difference = -difference;
      uint16_t threshold = (s_report[i].kind == TK_ANALOG) ? s_reportDelta : 1;
      if ((uint16_t) difference >= threshold) changed = true;
    }
  }

  if (s_reportDelta > 0 && !changed) return;   // delta mode: quiet until something moves

  Serial.print(F("!r"));
  for (uint8_t i = 0; i < s_reportCount; i++) {
    Serial.print(' ');
    if (s_report[i].kind == TK_ANALOG) { Serial.print('a'); Serial.print(s_report[i].index); }
    else                               { Serial.print('d'); Serial.print(s_report[i].pin); }
    Serial.print('=');
    Serial.print(sample[i]);
    s_report[i].last = sample[i];
  }
  Serial.print('\n');
  s_reportPrimed = true;
}

// ---------------------------------------------------------------- entry points
void setup() {
  memset(s_mode, MODE_NONE, sizeof(s_mode));
#if HAS_SERVO
  memset(s_servoPin, 255, sizeof(s_servoPin));
#endif

  Serial.begin(BAUD);
#if !defined(__AVR__)
  analogReadResolution(AIN_BITS);
#endif

  // Deliberately NOT waiting for the port (while (!Serial)): a native-USB board without
  // a host attached must still run. The announcement is lost when nobody listens — the
  // host's hello covers that case.
  sendReady();
}

void loop() {
  readSerial();
  reportTick();
  for (uint8_t m = 0; m < MOTOR_SLOTS; m++)
    motorTick(m);
}
