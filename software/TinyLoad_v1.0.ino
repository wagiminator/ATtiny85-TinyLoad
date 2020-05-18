
// tinyLoad
//
// Simple electronic dummy load. The ATtiny85 measures voltage, current
// and temperature of the heat sink, calculates power, energy and battery 
// capacity, controls the fan and displays all relevant data on the OLED.
// The button is used to switch between power/restistance and energy/capacity
// display.
//
// The ADC of the ATtiny does its best to make the tinyLoad a pretty accurate
// tool, but it might need a little calibration.
//
// How to calibrate:
// Set ULCAL and ILCAL to "1" in the sketch, compile and upload.
// Choose a stable input voltage of around 5V and turn the poti until the 
// display shows a current of around 0.7A. Measure the voltage and the current
// with a trusty multimeter or a good lab bench power supply.
// Calculate the voltage calibration factor as follows:
// ULCAL = voltage measured with multimeter / voltage shown on OLED.
// Calculate the current calibration factor as follows:
// ILCAL = current measured with multimeter / current shown on OLED.
// Set the ULCAL and ILCAL value in the sketch, compile and upload again.
//
// Notes:
// - Use a good heatsink with a 5V fan for the MOSFET !
// - Be careful with high power loads ! This device is called "tinyLoad" for a reason !
// - Always turn the POTI full counter-clockwise before connecting the load !
// - Due to the input offset voltage of the OpAmp the minimum load current is 17mA.
// - The maximum load current is 4.5A, however for small voltages it might be less.
// - Do not exceed the maximum voltage of 26V !
//
//                                 +-\/-+
// Button/NTC ------ A0 (D5) PB5  1|    |8  Vcc
// Voltage Sensor--- A3 (D3) PB3  2|    |7  PB2 (D2) A1 ---- OLED (SCK)
// Current Sensor -- A2 (D4) PB4  3|    |6  PB1 (D1) ------- Fan
//                           GND  4|    |5  PB0 (D0) ------- OLED (SDA)
//                                 +----+
//
// RESET pin is used as a weak analog input for the set button and the NTC
// thermistor. You don't need to disable the RESET pin as the voltage won't 
// go below 40% of Vcc.
//
// Controller:  ATtiny85
// Core:        attiny by David A. Mellis (https://github.com/damellis/attiny)
// Clockspeed:  16 MHz internal
//
// 2020 by Stefan Wagner (https://easyeda.com/wagiminator)
// License: http://creativecommons.org/licenses/by-sa/3.0/


// Libraries
#include <TinyWireM.h>          // https://github.com/adafruit/TinyWireM
#include <Tiny4kOLED.h>         // https://github.com/datacute/Tiny4kOLED
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

// Calibration values for internal 1.1V reference
#define ULCAL         1         // linear voltage calibration factor
#define ILCAL         1         // linear current calibration factor

// Parameter definitions
#define MAXPOWER      15        // max power in watt
#define UCONV         11486336  // voltage conversion = 1000 * 64 * 1023 * R11 / (R10 + R11)
#define ICONV         72019200  // current conversion = 1000 * 64 * 1023 * R_shunt * (R8 + R9) / R9

// RESET pin ADC values (tweak a little, if button/NTC do not work correctly)
#define NTCMIN        550       // lowest ADC value for NTC; below is a button press
#define NTCFAN        625       // turn on the fan if ADC value is below
#define NTCHOT        560       // show warning screen if ADC value is below

// Voltage references
#define REFVCC        0                             // Vcc as reference
#define REF1V1        bit(REFS1)                    // internal 1.1V reference
#define REF2V56       bit(REFS2) | bit (REFS1)      // internal 2.56V reference
#define AVBG          bit (MUX3) | bit (MUX2)       // port of internal bandgap (1.1V reference)

// Pin assignments
#define FAN_PIN       1
#define NTC_PIN       A0
#define CURRENT_PIN   A2
#define VOLTAGE_PIN   A3


// Text strings stored in program memory
const char StartScreen[]  PROGMEM = 
  "-- TinyLoad  v1.0  --"
  "Turn POTI full CCW,  "
  "connect the load and "
  "press SET button.    ";
const char WarnScreen[]  PROGMEM =
  "!!!!!  WARNING  !!!!!"
  "You reached max power"
  "or temperature !     "
  "!!! DECREASE LOAD !!!";

// Variables
uint16_t  vcc, vref2, onevolt, twovolt, rawCheck, temp;
float     rawVoltage, rawCurrent, voltage, current, power, resistance;
float     capacity = 0;
float     energy = 0;
bool      primescreen = true;
bool      warnscreen;
bool      lastbutton;
uint32_t  lastmillis, nowmillis, interval;


void setup() {
  // setup Pins
  DDRB  = bit (FAN_PIN);                              // set output pins
  PORTB = 0;                                          // fan off, no pullups

  // setup ADC
  ADCSRA  = bit (ADPS0) | bit (ADPS1) | bit (ADPS2);  // set ADC clock prescaler to 128
  ADCSRA |= bit (ADEN)  | bit (ADIE);                 // enable ADC and ADC interrupt                          
  interrupts ();                                      // enable global interrupts

  // calibrate internal 2.56V reference
  vref2 = getADC(AVBG, REF2V56, 64);                  // read internal bandgap against 2.56V reference
  vref2 = (uint32_t)72019200 / vref2;                 // calculate Vref2 in mV; 72019200 = 64*1023*1100

  // prepare and start OLED
  oled.begin();
  oled.setFont(FONT6X8);
  oled.clear();
  oled.on();
  oled.switchRenderFrame();

  // check if something is connected to the load
  if ( !getADC(VOLTAGE_PIN, REFVCC, 1) ) {
    oled.clear(); oled.setCursor(0, 0); printP(StartScreen);
    oled.switchFrame();
    while( (!buttonPressed()) && (getADC(VOLTAGE_PIN, REFVCC, 1) < 5) );
  }

  // init some variables;
  lastmillis = millis();
  lastbutton = true;
}


void loop() {
  vcc     = getADC(AVBG, REFVCC, 64);                 // read internal bandgap against Vcc with 64 samples
  vcc     = (uint32_t)72019200 / vcc;                 // calculate Vcc in mV; 1125300 = 64*1023*1100
  onevolt = (uint32_t)1125300 / vcc;                  // calculate ADC-value for 1.1V
  twovolt = (uint32_t)vref2 * 1023 / vcc;             // calculate ADC-value for 2.56V (calibrated)

  // read and calculate voltage in V
  rawCheck = getADC(VOLTAGE_PIN, REFVCC, 1);          // get ADC-value for voltage reading
  if (rawCheck < onevolt) {                           // check possibility of reading against 1.1V reference
    rawVoltage  = getADC(VOLTAGE_PIN, REF1V1, 64);    // read raw voltage against 1.1V reference
    voltage     = rawVoltage * 1100 / UCONV * ULCAL;  // calculate real voltage value
  }
  else if (rawCheck < twovolt) {                      // check possibility of reading against 2.56V reference
    rawVoltage  = getADC(VOLTAGE_PIN, REF2V56, 64);   // read raw voltage against 2.56V reference
    voltage     = rawVoltage * vref2 / UCONV * ULCAL; // calculate real voltage value
  }
  else {
    rawVoltage  = getADC(VOLTAGE_PIN, REFVCC, 64);    // read raw voltage against vcc
    voltage     = rawVoltage * vcc / UCONV * ULCAL;   // calculate real voltage value
  }

  // read and calculate current in A
  rawCheck = getADC(CURRENT_PIN, REFVCC, 1);          // get ADC-value for current reading
  if (rawCheck < onevolt) {                           // check possibility of reading against 1.1V reference
    rawCurrent  = getADC(CURRENT_PIN, REF1V1, 64);    // read raw current against 1.1V reference
    current     = rawCurrent * 1100 / ICONV * ILCAL;  // calculate real current value
  }
  else if (rawCheck < twovolt) {                      // check possibility of reading against 2.56V reference
    rawCurrent  = getADC(CURRENT_PIN, REF2V56, 64);   // read raw current against 2.56V reference
    current     = rawCurrent * vref2 / ICONV * ILCAL; // calculate real current value
  }
  else {
    rawCurrent  = getADC(CURRENT_PIN, REFVCC, 64);    // read raw current against vcc
    current     = rawCurrent * vcc / ICONV * ILCAL;   // calculate real current value
  }
  
  // calculate power and resistance
  power       = voltage * current;                    // calculate power in W
  if (current > 0) resistance = voltage / current;    // calculate resistance in Ohm
  else resistance = 9999;                             // 9999 if no current
  if (resistance > 9999) resistance = 9999;           // 9999 is max displayed value

  // calculate energy/capacity in mWh and mAh
  nowmillis   = millis();
  interval    = nowmillis - lastmillis;               // calculate time interval
  lastmillis  = nowmillis;
  energy   += power   * interval / 3600;              // calculate mWh
  capacity += current * interval / 3600;              // calculate mAh

  // turn on fan if needed
  temp = getADC(NTC_PIN, REFVCC, 1);
  if ( (power > 1) || ( (temp >= NTCMIN) && (temp < NTCFAN) ) ) bitSet(PORTB, FAN_PIN);
  else bitClear(PORTB, FAN_PIN);

  // check limits
  warnscreen = false;
  if ( (power > MAXPOWER) || ( (temp >= NTCMIN) && (temp < NTCHOT) ) ) warnscreen = true;

  // check button
  if (!lastbutton && buttonPressed()) primescreen = !primescreen;
  lastbutton = buttonPressed();
   
  // display values on the oled
  oled.clear();
  oled.setCursor(0, 0);

  if (warnscreen) printP(WarnScreen);
  else {
    oled.print(F("Voltage:   ")); printValue(voltage); oled.print(F(" V"));
    oled.setCursor(0, 1);
    oled.print(F("Current:   ")); printValue(current); oled.print(F(" A"));

    if (primescreen) {
      oled.setCursor(0, 2);
      oled.print(F("Power:     ")); printValue(power);      oled.print(F(" W"));
      oled.setCursor(0, 3);
      oled.print(F("Resistance:")); printValue(resistance); oled.print(F(" O"));
    } else {
      oled.setCursor(0, 2);
      oled.print(F("Energy:  ")); printValue(energy);   oled.print(F(" mWh"));
      oled.setCursor(0, 3);
      oled.print(F("Capacity:")); printValue(capacity); oled.print(F(" mAh"));
    }
  } 
  oled.switchFrame();
}


// returns true if button is pressed
boolean buttonPressed() {
  uint16_t value = getADC(NTC_PIN, REFVCC, 1);
  return (value < NTCMIN);
}


// returns true if NTC is connected
boolean NTCpresent() {
  uint16_t value = getADC(NTC_PIN, REFVCC, 1);
  return ( (value >= NTCMIN) && (value < 1020) );
}


// prints a string from progmem on the OLED
void printP(const char* p) {
  char ch = pgm_read_byte(p);
  while (ch != 0) {
    oled.print(ch);
    ch = pgm_read_byte(++p);
  }
}


// prints value right aligned
void printValue(float value) {
  value += 0.005;
  uint32_t counter = value;
  if (counter == 0) counter = 1;
  while (counter < 10000) {
    oled.print(" ");
    counter *= 10;
  }
  oled.print(value, 2);
}


// add up several ADC readings (oversampling) against choosen reference in sleep mode to denoise
// max samples: 64
uint16_t getADC (uint8_t port, uint8_t reference, uint8_t samples) {
  uint16_t result = 0;                      // result
  ADCSRA |= bit (ADEN) | bit (ADIF);        // enable ADC, turn off any pending interrupt
  if ((port<0x0c)&&(port>=A0)) port -= A0;  // convert port to analog channel selection bits
  ADMUX = (port & 0x0f) | reference;        // set input channel and reference
  if (reference || (port==0x0c)) delay(1);  // wait 1ms to settle if Vref is used  
  set_sleep_mode (SLEEP_MODE_ADC);          // sleep during sample for noise reduction
  for (uint8_t i=0; i<samples; i++) {       // get several samples
    sleep_mode();                           // go to sleep while taking ADC sample
    while (bitRead(ADCSRA, ADSC));          // make sure sampling is completed
    result += ADC;                          // add them up
  }
  return (result);                          // return value
}


// ADC interrupt service routine
EMPTY_INTERRUPT (ADC_vect);                 // nothing to be done here
