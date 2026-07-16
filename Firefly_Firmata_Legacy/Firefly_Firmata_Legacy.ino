/*
 Firefly Firmata Legacy (the original Firefly Firmata, 2015-2026)
 Created by Andrew Payne
 Latest Update July 2026 (Firefly 2 maintenance pass; wire protocol unchanged since 2015)
 Copyright 2015-2026 Andrew Payne
 License: MIT (see the LICENSE file in this repository)

 This Firmata allows you to control an Arduino board from Rhino/Grasshopper/Firefly.
 Updates, Questions, Suggestions visit: http://www.fireflyexperiments.com

 1. Plug Arduino boards into your USB port; confirm that your Arduino's green power LED in on
 2. Select your specific Arduino Board and Serial Port (Tools > Board; Tools > Serial Port) *Take note of your Serial Port COM #
 3. Verify (play button) and Upload (upload button) this program to your Arduino, close the Arduino program
 4. then open ... Rhino/Grasshopper/Firefly

 Note: The Firefly Firmata sets the following pins to perform these functions:

 *****ON STANDARD BOARDS (ie. Uno, Diecimila, Duemilanove, Lillypad, Mini, etc.)*****
 ANALOG IN pins 0-5 are set to return values (from 0 to 1023) for analog sensors
 DIGITAL IN pins 2,4,7 will return 0's or 1's; for 3 potential digital sensors (buttons, switches, on/off, true/false, etc.)
 DIGITAL/ANALOG OUT pins 3,5,6,11 (marked with a ~) can be used to digitalWrite, analogWrite, or Servo.write depending on the input status of that Firefly pin
 DIGITAL OUT pins 8,9,10,12,13 can be used to digitalWrite or Servo.write depending on the input status of that Firefly pin

 *****ON MEGA BOARDS (ie. ATMEGA1280, ATMEGA2560)*****
 ANALOG IN pins 0-15 will return values (from 0 to 1023) for 16 analog sensors
 DIGITAL IN pins 22-31 will return 0's or 1's; for digital sensors (buttons, switches, on/off, true/false, etc.)
 DIGITAL/ANALOG OUT pins 2-13 can be used to digitalWrite, analogWrite, or Servo.write depending on the input status of that Firefly pin
 DIGITAL OUT pins 32-53 can be used to digitalWrite, Servo.write, or analogWrite depending on the input status of that Firefly pin

 *****ON LEONARDO BOARDS*****
 ANALOG IN pins 0-5 are set to return values (from 0 to 1023) for analog sensors
 DIGITAL IN pins 2,4,7 will return 0's or 1's; for 3 potential digital sensors (buttons, switches, on/off, true/false, etc.)
 DIGITAL/ANALOG OUT pins 3,5,6,11 (marked with a ~) can be used to digitalWrite, analogWrite, or Servo.write depending on the input status of that Firefly pin
 DIGITAL OUT pins 8,9,10,12,13 can be used to digitalWrite or Servo.write depending on the input status of that Firefly pin

  *****ON DUE BOARDS (ie. SAM3X8E)*****
 ANALOG IN pins 0-11 will return values (from 0 to 4095) for 12 analog sensors
 DIGITAL IN pins 22-31 will return 0's or 1's; for digital sensors (buttons, switches, on/off, true/false, etc.)
 DIGITAL/ANALOG OUT pins 2-13 can be used to digitalWrite, analogWrite, or Servo.write depending on the input status of that Firefly pin
 DIGITAL OUT pins 32-53 can be used to digitalWrite, Servo.write, or analogWrite depending on the input status of that Firefly pin
 DAC0 and DAC1 can be used to output an analog voltage on those pins (only available on DUE boards)

 *****WIRE PROTOCOL (do not change without updating Firefly)*****
 Board -> Grasshopper: "a0,a1,...,d0,d1,...,eol\n" — one reading per input pin, then "eol".
 Grasshopper -> board: one integer per WRITE_PIN_CONFIG entry, comma separated, "\n" terminated.
 Each value is range-encoded: 10000+v digitalWrite (v 0-1), 20000+v analogWrite (v 0-255),
 30000+v Servo.write (v 0-180), 40000+v DAC (v 0-4095, DUE only). Values below 10000 leave
 the pin untouched.
 */

#include <Servo.h>            // Servo library (bundled with the Arduino IDE)
#include <pins_arduino.h>     // board pin definitions, used to detect the board type

#define BAUDRATE 115200       // must match the Baud input on the Firefly components
#define BUFFSIZE 512          // incoming line buffer (one command per line)
#define SEND_INTERVAL_MS 10   // how often input readings are streamed out (~100 Hz)

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

char buffer[BUFFSIZE];            // incoming line buffer
uint16_t bufferidx = 0;           // write position within the buffer
bool discardline = false;         // when true, drop everything until the next newline

unsigned long lastsend = 0;       // millis() timestamp of the last input report

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)                                                // declare variables for STANDARD boards
  Servo Servo13, Servo12, Servo11, Servo10, Servo9, Servo8, Servo6, Servo5, Servo3;
  Servo SERVO_CONFIG[] = {Servo13, Servo12, Servo11, Servo10, Servo9, Servo8, Servo6, Servo5, Servo3};       // declare array of Servo objects
  int WRITE_PIN_CONFIG[] = {13,12,11,10,9,8,6,5,3};
  int READ_APIN_CONFIG[] = {0,1,2,3,4,5};
  int READ_DPIN_CONFIG[] = {2,4,7};
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)                                               // declare variables for LEONARDO board
  Servo Servo13, Servo12, Servo11, Servo10, Servo9, Servo8, Servo6, Servo5, Servo3;
  Servo SERVO_CONFIG[] = {Servo13, Servo12, Servo11, Servo10, Servo9, Servo8, Servo6, Servo5, Servo3};       // declare array of Servo objects
  int WRITE_PIN_CONFIG[] = {13,12,11,10,9,8,6,5,3};
  int READ_APIN_CONFIG[] = {0,1,2,3,4,5};
  int READ_DPIN_CONFIG[] = {2,4,7};
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)                        // declare variables for MEGA boards
  Servo Servo2, Servo3, Servo4, Servo5, Servo6, Servo7, Servo8, Servo9, Servo10, Servo11, Servo12, Servo13, Servo32, Servo33, Servo34, Servo35, Servo36, Servo37, Servo38, Servo39, Servo40, Servo41, Servo42, Servo43, Servo44, Servo45, Servo46, Servo47, Servo48, Servo49, Servo50, Servo51, Servo52, Servo53;
  Servo SERVO_CONFIG[] = {Servo2, Servo3, Servo4, Servo5, Servo6, Servo7, Servo8, Servo9, Servo10, Servo11, Servo12, Servo13, Servo32, Servo33, Servo34, Servo35, Servo36, Servo37, Servo38, Servo39, Servo40, Servo41, Servo42, Servo43, Servo44, Servo45, Servo46, Servo47, Servo48, Servo49, Servo50, Servo51, Servo52, Servo53};  // declare array of Servo objects
  int WRITE_PIN_CONFIG[] = {2,3,4,5,6,7,8,9,10,11,12,13,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53};
  int READ_APIN_CONFIG[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  int READ_DPIN_CONFIG[] = {22,23,24,25,26,27,28,29,30,31};
#endif

#if defined(__SAM3X8E__)                 // declare variables for DUE boards
  Servo FDAC0, FDAC1, Servo2, Servo3, Servo4, Servo5, Servo6, Servo7, Servo8, Servo9, Servo10, Servo11, Servo12, Servo13, Servo32, Servo33, Servo34, Servo35, Servo36, Servo37, Servo38, Servo39, Servo40, Servo41, Servo42, Servo43, Servo44, Servo45, Servo46, Servo47, Servo48, Servo49, Servo50, Servo51, Servo52, Servo53;
  Servo SERVO_CONFIG[] = {FDAC0, FDAC1, Servo2, Servo3, Servo4, Servo5, Servo6, Servo7, Servo8, Servo9, Servo10, Servo11, Servo12, Servo13, Servo32, Servo33, Servo34, Servo35, Servo36, Servo37, Servo38, Servo39, Servo40, Servo41, Servo42, Servo43, Servo44, Servo45, Servo46, Servo47, Servo48, Servo49, Servo50, Servo51, Servo52, Servo53};  // declare array of Servo objects
  int WRITE_PIN_CONFIG[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53}; //Note: first two values correspond to the DAC pins
  int READ_APIN_CONFIG[] = {0,1,2,3,4,5,6,7,8,9,10,11};
  int READ_DPIN_CONFIG[] = {22,23,24,25,26,27,28,29,30,31};
#endif

/*==============================================================================
 * SETUP() This code runs once
 *============================================================================*/
void setup()
{
  Init();                       // set initial pinmodes
  Serial.begin(BAUDRATE);       // Start Serial communication
  #if defined(__SAM3X8E__)      // if the connected board is an Arduino DUE
    analogReadResolution(12);   // Set the analog read resolution to 12 bits (acceptable values between 1-32 bits).  This is only for DUE boards
    analogWriteResolution(12);  // Set the analog write resolution to 12 bits (acceptable values between 1-32 bits).  This is only for DUE boards
  #endif
}

/*==============================================================================
 * LOOP() This code loops
 *============================================================================*/
void loop()
{
  if (Serial) {
    ReadSerial();                                     // drain and parse everything waiting on the serial port
    unsigned long now = millis();
    if (now - lastsend >= SEND_INTERVAL_MS) {         // stream input readings on a wall-clock schedule
      ReadInputs();                                   // (the old version counted loop iterations, which made
      lastsend = now;                                 //  the report rate vary with board speed and load)
    }
  }
}

/*==============================================================================
 * FUNCTIONS()
 *============================================================================*/

/*
* Initializes the digital pins which will be used as inputs
*/
void Init(){
  int len = sizeof(READ_DPIN_CONFIG)/sizeof(READ_DPIN_CONFIG[0]); //get the size of the array
  for(int i = 0; i < len; i++){
    pinMode(READ_DPIN_CONFIG[i], INPUT);
  }
}

/*
* Reads the incoming ADC or digital values from the corresponding analog and digital input
* pins and prints the value to the serial port as a formatted comma separated string
*/
void ReadInputs(){
  int len = sizeof(READ_APIN_CONFIG)/sizeof(READ_APIN_CONFIG[0]); //get the size of the array
  for(int i = 0; i < len; i++){
    int val = analogRead(READ_APIN_CONFIG[i]);  //read value from analog pins
    Serial.print(val); Serial.print(",");
  }
  len = sizeof(READ_DPIN_CONFIG)/sizeof(READ_DPIN_CONFIG[0]); //get the size of the array
  for(int i = 0; i < len; i++){
    int val = digitalRead(READ_DPIN_CONFIG[i]); //read value from digital pins
    Serial.print(val); Serial.print(",");
  }
  Serial.println("eol");  //end of line marker
}

/*
* Drains the serial receive buffer, accumulating characters until a newline completes a
* command line, then parses it. Lines longer than the buffer are discarded in one piece
* (rather than wrapping and corrupting the next command, as the old version did).
*/
void ReadSerial(){
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (!discardline && bufferidx > 0) {
        buffer[bufferidx] = 0;         // terminate the line
        ParseLine();                   // parse it and write the values out to the pins
      }
      bufferidx = 0;                   // start the next line fresh either way
      discardline = false;
    }
    else if (discardline) {
      // skip everything up to the next newline
    }
    else if (bufferidx < BUFFSIZE - 1) {
      buffer[bufferidx++] = c;
    }
    else {
      discardline = true;              // line too long: drop it entirely
    }
  }
}

/*
* Splits the completed command line at the comma delimiters and writes each value to the
* corresponding output pin. Malformed or short lines stop at the last valid value instead
* of reading past the end of the buffer.
*/
void ParseLine(){
  char *parseptr = buffer;
  int len = sizeof(WRITE_PIN_CONFIG)/sizeof(WRITE_PIN_CONFIG[0]); //get the size of the array
  for(int i = 0; i < len; i++){
    long val = parsedecimal(parseptr);                   // parse the incoming number
    WriteToPin(WRITE_PIN_CONFIG[i], val, SERVO_CONFIG[i]); //send value out to pin on arduino board
    if(i != len - 1){
      parseptr = strchr(parseptr, ',');                  // find the next ","
      if (parseptr == NULL) return;                      // short line: leave remaining pins untouched
      parseptr++;                                        // move past the ","
    }
  }
}

/*
* Send the incoming value to the appropriate pin using pre-defined logic (ie. digital, analog, or servo)
*/
void WriteToPin(int _pin, long _value, Servo &_servo){
  if (_value >= 10000 && _value < 20000)            // check if value should be used for Digital Write (HIGH/LOW)
  {
    if (_servo.attached()) _servo.detach();         // detach servo if one is attached to pin
    pinMode(_pin, OUTPUT);
    _value -= 10000;                                // subtract 10,000 from the value sent from Grasshopper
    if (_value == 1) digitalWrite(_pin, HIGH);
    else digitalWrite(_pin, LOW);
  }
  else if (_value >= 20000 && _value < 30000)       // check if value should be used for Analog Write (0-255)
  {
    if (_servo.attached()) _servo.detach();         // detach servo if one is attached to pin
    pinMode(_pin, OUTPUT);
    _value -= 20000;                                // subtract 20,000 from the value sent from Grasshopper
    analogWrite(_pin, _value);
  }
  else if (_value >= 30000 && _value < 40000)       // check if value should be used for Servo Write (0-180)
  {
    _value -= 30000;                                // subtract 30,000 from the value sent from Grasshopper
    if (!_servo.attached())_servo.attach(_pin);     // attaches a Servo to the PWM pin (180 degree standard servos)
    _servo.write(_value);
  }
#if defined(__SAM3X8E__)                            // DAC output only exists on DUE boards. NB: without this
  else if (_value >= 40000 && _value < 50000)       // guard the sketch does not COMPILE on Uno/Mega/Leonardo
  {                                                 // with modern toolchains (WriteToDAC is undeclared there).
    if (_servo.attached()) _servo.detach();         // detach servo if one is attached to pin
    pinMode(_pin, OUTPUT);
    _value -= 40000;                                // subtract 40,000 from the value sent from Grasshopper
    WriteToDAC(_pin, _value);
  }
#endif
}

/*
* Parse the leading digits of a string as a decimal number. Stops at the first non-digit
* (the old version compared against the multi-character literal '50', which accepted ':'
* through the end of the ASCII digits by accident).
*/
long parsedecimal(char *str){
  long d = 0;
  while (*str >= '0' && *str <= '9') {
    d = d * 10 + (*str - '0');
    str++;
  }
  return d;
}

/*
* Send the incoming value to the appropriate DAC for DUE boards.
* Note: analogWrite resolution (default is 12 bits) is defined in the Setup function.
*/
 #if defined(__SAM3X8E__)
  void WriteToDAC(int _pin, long _value){
    if(_pin == 0) analogWrite(DAC0, _value);
    else if (_pin == 1) analogWrite(DAC1, _value);
  }
#endif
