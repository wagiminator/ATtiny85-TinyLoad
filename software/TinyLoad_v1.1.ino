// tinyLoad - Simple Electronic Constant Current Dummy Load based on ATtiny45/85
//
// The ATtiny measures voltage, current and temperature of the heat sink,
// calculates power, energy and electric charge, controls the fan and displays
// all relevant data on the OLED. The button is used to switch between
// power/resistance and energy/charge display.
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
// Button/NTC ------ A0 (D5) PB5  1|°   |8  Vcc
// Voltage Sensor--- A3 (D3) PB3  2|    |7  PB2 (D2) A1 ---- OLED (SCK)
// Current Sensor -- A2 (D4) PB4  3|    |6  PB1 (D1) ------- Fan
//                           GND  4|    |5  PB0 (D0) ------- OLED (SDA)
//                                 +----+
//
// RESET pin is used as a weak analog input for the set button and the NTC
// thermistor. You don't need to disable the RESET pin as the voltage won't 
// go below 40% of Vcc.
//
// Core:    ATtinyCore (https://github.com/SpenceKonde/ATTinyCore)
// Board:   ATtiny25/45/85 (No bootloader)
// Chip:    ATtiny45 or 85 (depending on your chip)
// Clock:   8 MHz (internal)
// Millis:  disabled
// B.O.D.:  2.7V
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// No Arduino core functions or libraries are used. Use the makefile if 
// you want to compile without Arduino IDE.
//
// The I²C OLED implementation is based on TinyOLEDdemo
// https://github.com/wagiminator/ATtiny13-TinyOLEDdemo
//
// OLED font was adapted from Neven Boyanov and Stephen Denne
// https://github.com/datacute/Tiny4kOLED
//
// 2020 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/


// Libraries
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Pin assignments
#define I2C_SDA       PB0       // I2C serial data pin
#define I2C_SCL       PB2       // I2C serial clock pin
#define FAN_PIN       PB1       // Fan control
#define NTC_AP        0         // ADC port of fan pin
#define CURRENT_AP    2         // ADC port of current sense pin
#define VOLTAGE_AP    3         // ADC port of voltage sense pin

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

// -----------------------------------------------------------------------------
// I2C Implementation
// -----------------------------------------------------------------------------

// I2C macros
#define I2C_SDA_HIGH()  DDRB &= ~(1<<I2C_SDA)   // release SDA   -> pulled HIGH by resistor
#define I2C_SDA_LOW()   DDRB |=  (1<<I2C_SDA)   // SDA as output -> pulled LOW  by MCU
#define I2C_SCL_HIGH()  DDRB &= ~(1<<I2C_SCL)   // release SCL   -> pulled HIGH by resistor
#define I2C_SCL_LOW()   DDRB |=  (1<<I2C_SCL)   // SCL as output -> pulled LOW  by MCU
#define I2C_SDA_READ()  (PINB  &  (1<<I2C_SDA)) // read SDA line
#define I2C_DELAY()     asm("lpm")              // delay 3 clock cycles
#define I2C_CLOCKOUT()  I2C_SCL_HIGH();I2C_DELAY();I2C_SCL_LOW()  // clock out

// I2C init function
void I2C_init(void) {
  DDRB  &= ~((1<<I2C_SDA)|(1<<I2C_SCL));      // pins as input (HIGH-Z) -> lines released
  PORTB &= ~((1<<I2C_SDA)|(1<<I2C_SCL));      // should be LOW when as ouput
}

// I2C transmit one data byte to the slave, ignore ACK bit, no clock stretching allowed
void I2C_write(uint8_t data) {
  for(uint8_t i=8; i; i--, data<<=1) {        // transmit 8 bits, MSB first
    (data & 0x80) ? (I2C_SDA_HIGH()) : (I2C_SDA_LOW());  // SDA HIGH if bit is 1
    I2C_CLOCKOUT();                           // clock out -> slave reads the bit
  }
  I2C_DELAY();                                // delay 3 clock cycles
  I2C_SDA_HIGH();                             // release SDA for ACK bit of slave
  I2C_CLOCKOUT();                             // 9th clock pulse is for the ignored ACK bit
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  I2C_SDA_LOW();                              // start condition: SDA goes LOW first
  I2C_SCL_LOW();                              // start condition: SCL goes LOW second
  I2C_write(addr);                            // send slave address
}

// I2C stop transmission
void I2C_stop(void) {
  I2C_SDA_LOW();                              // prepare SDA for LOW to HIGH transition
  I2C_SCL_HIGH();                             // stop condition: SCL goes HIGH first
  I2C_SDA_HIGH();                             // stop condition: SDA goes HIGH second
}

// -----------------------------------------------------------------------------
// OLED Implementation
// -----------------------------------------------------------------------------

// OLED definitions
#define OLED_ADDR       0x78              // OLED write address
#define OLED_CMD_MODE   0x00              // set command mode
#define OLED_DAT_MODE   0x40              // set data mode
#define OLED_INIT_LEN   9                 // length of init command array

// OLED 5x8 pixels character set
const uint8_t OLED_FONT[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00,
  0x14, 0x7F, 0x14, 0x7F, 0x14, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62,
  0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x1C, 0x22, 0x41, 0x00,
  0x00, 0x41, 0x22, 0x1C, 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14, 0x08, 0x08, 0x3E, 0x08, 0x08,
  0x00, 0x00, 0xA0, 0x60, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x60, 0x60, 0x00, 0x00,
  0x20, 0x10, 0x08, 0x04, 0x02, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00,
  0x42, 0x61, 0x51, 0x49, 0x46, 0x21, 0x41, 0x45, 0x4B, 0x31, 0x18, 0x14, 0x12, 0x7F, 0x10,
  0x27, 0x45, 0x45, 0x45, 0x39, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x01, 0x71, 0x09, 0x05, 0x03,
  0x36, 0x49, 0x49, 0x49, 0x36, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x36, 0x36, 0x00, 0x00,
  0x00, 0x56, 0x36, 0x00, 0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x00, 0x41, 0x22, 0x14, 0x08, 0x02, 0x01, 0x51, 0x09, 0x06, 0x32, 0x49, 0x59, 0x51, 0x3E,
  0x7C, 0x12, 0x11, 0x12, 0x7C, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E, 0x41, 0x41, 0x41, 0x22,
  0x7F, 0x41, 0x41, 0x22, 0x1C, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x7F, 0x09, 0x09, 0x09, 0x01,
  0x3E, 0x41, 0x49, 0x49, 0x7A, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x41, 0x7F, 0x41, 0x00,
  0x20, 0x40, 0x41, 0x3F, 0x01, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x7F, 0x40, 0x40, 0x40, 0x40,
  0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x4E, 0x71, 0x01, 0x71, 0x4E,
  0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x7F, 0x09, 0x19, 0x29, 0x46,
  0x46, 0x49, 0x49, 0x49, 0x31, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x3F, 0x40, 0x40, 0x40, 0x3F,
  0x1F, 0x20, 0x40, 0x20, 0x1F, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x63, 0x14, 0x08, 0x14, 0x63,
  0x07, 0x08, 0x70, 0x08, 0x07, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x7F, 0x41, 0x41, 0x00,
  0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x41, 0x41, 0x7F, 0x00, 0x04, 0x02, 0x01, 0x02, 0x04,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x01, 0x02, 0x04, 0x00, 0x20, 0x54, 0x54, 0x54, 0x78,
  0x7F, 0x48, 0x44, 0x44, 0x38, 0x38, 0x44, 0x44, 0x44, 0x20, 0x38, 0x44, 0x44, 0x48, 0x7F,
  0x38, 0x54, 0x54, 0x54, 0x18, 0x08, 0x7E, 0x09, 0x01, 0x02, 0x18, 0xA4, 0xA4, 0xA4, 0x7C,
  0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00,
  0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78,
  0x7C, 0x08, 0x04, 0x04, 0x78, 0x38, 0x44, 0x44, 0x44, 0x38, 0xFC, 0x24, 0x24, 0x24, 0x18,
  0x18, 0x24, 0x24, 0x18, 0xFC, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x48, 0x54, 0x54, 0x54, 0x20,
  0x04, 0x3F, 0x44, 0x40, 0x20, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x1C, 0x20, 0x40, 0x20, 0x1C,
  0x3C, 0x40, 0x30, 0x40, 0x3C, 0x44, 0x28, 0x10, 0x28, 0x44, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C,
  0x44, 0x64, 0x54, 0x4C, 0x44, 0x08, 0x36, 0x41, 0x41, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00,
  0x00, 0x41, 0x41, 0x36, 0x08, 0x08, 0x04, 0x08, 0x10, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// OLED init settings
const uint8_t OLED_INIT_CMD[] PROGMEM = {
  0xC8, 0xA1,   // flip screen
  0xA8, 0x1F,   // set multiplex ratio
  0xDA, 0x02,   // set com pins hardware configuration
  0x8D, 0x14,   // set DC-DC enable
  0xAF          // display on
};

// OLED variables
uint8_t OLED_x, OLED_y;                   // current cursor position

// OLED init function
void OLED_init(void) {
  I2C_init();                             // initialize I2C first
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  for (uint8_t i = 0; i < OLED_INIT_LEN; i++)
    I2C_write(pgm_read_byte(&OLED_INIT_CMD[i])); // send the command bytes
  I2C_stop();                             // stop transmission
}

// OLED set the cursor
void OLED_setCursor(uint8_t xpos, uint8_t ypos) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  I2C_write(xpos & 0x0F);                 // set low nibble of start column
  I2C_write(0x10 | (xpos >> 4));          // set high nibble of start column
  I2C_write(0xB0 | (ypos & 0x07));        // set start page
  I2C_stop();                             // stop transmission
  OLED_x = xpos; OLED_y = ypos;           // set the cursor variables
}

// OLED clear line
void OLED_clearLine(uint8_t line) {
  OLED_setCursor(0, line);                // set cursor to line start
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  for(uint8_t i=128; i; i--) I2C_write(0x00); // clear the line
  I2C_stop();                             // stop transmission
}

// OLED clear screen
void OLED_clearScreen(void) {
  for(uint8_t i=0; i<4; i++)              // 4 lines
    OLED_clearLine(i);                    // clear line
}

// OLED plot a single character
void OLED_plotChar(char c) {
  uint16_t ptr = c - 32;                  // character pointer
  ptr += ptr << 2;                        // -> ptr = (ch - 32) * 5;
  I2C_write(0x00);                        // write space between characters
  for (uint8_t i=5 ; i; i--) I2C_write(pgm_read_byte(&OLED_FONT[ptr++]));
  OLED_x += 6;                            // update cursor
  if (OLED_x > 122) {                     // line end ?
    I2C_stop();                           // stop data transmission
    OLED_setCursor(0,++OLED_y);           // set next line start
    I2C_start(OLED_ADDR);                 // start transmission to OLED
    I2C_write(OLED_DAT_MODE);             // set data mode
  }
}

// OLED print a string from program memory
void OLED_printPrg(const char* p) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  char ch = pgm_read_byte(p);             // read first character from program memory
  while (ch) {                            // repeat until string terminator
    OLED_plotChar(ch);                    // print character on OLED
    ch = pgm_read_byte(++p);              // read next character
  }
  I2C_stop();                             // stop transmission
}

// OLED print 8-bit value as 2-digit decimal (BCD conversion by substraction method)
void OLED_printDec8(uint8_t value) {
  uint8_t digitval = 0;                   // start with digit value 0
  while (value >= 10) {                   // if current divider fits into the value
    digitval++;                           // increase digit value
    value -= 10;                          // decrease value by divider
  }
  OLED_plotChar(digitval + '0');          // print first digit
  OLED_plotChar(value + '0');             // print second digit
}

// OLED print 16-bit value as decimal (BCD conversion by substraction method)
void OLED_printDec16(uint16_t value) {
  static uint16_t divider[5] = {10000, 1000, 100, 10, 1};  // for BCD conversion
  uint8_t leadflag = 0;
  for(uint8_t digit = 0; digit < 5; digit++) {  // 5 digits
    uint8_t digitval = 0;                 // start with digit value 0
    while (value >= divider[digit]) {     // if current divider fits into the value
      leadflag = 1;                       // end of leading spaces
      digitval++;                         // increase digit value
      value -= divider[digit];            // decrease value by divider
    }
    if (leadflag || (digit == 4)) OLED_plotChar(digitval + '0'); // print the digit
    else OLED_plotChar(' ');              // print leading space
  }
}

// OLED print float right-aligned
void OLED_printValue(float value) {
  value += 0.005;                         // for rounding
  uint16_t front = value;                 // left side of decimal
  uint8_t back = (value - front) * 100 ;  // right side of decimal
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  OLED_printDec16(front);                 // print left side of decimal
  OLED_plotChar('.');                     // print decimal
  OLED_printDec8(back);                   // print right side of decimal
  I2C_stop();                             // stop transmission to OLED
}

// -----------------------------------------------------------------------------
// Millis Counter Implementation for Timer0
// -----------------------------------------------------------------------------

volatile uint32_t MIL_counter = 0;  // millis counter variable

// Init millis counter
void MIL_init(void) {
  OCR0A  = 124;                     // TOP: 124 = 8000kHz / (64 * 1kHz) - 1
  TCCR0A = (1<<WGM01);              // timer0 CTC mode
  TCCR0B = (1<<CS01)|(1<<CS00);     // start timer0 with prescaler 64
  TIMSK  = (1<<OCIE0A);             // enable output compare match interrupt
}

// Read millis counter
uint32_t MIL_read(void) {
  cli();                            // disable interrupt for atomic read
  uint32_t result = MIL_counter;    // read millis counter
  sei();                            // enable interrupts
  return(result);                   // return millis counter value
}

// Timer0 compare match A interrupt service routine (every millisecond)
ISR(TIM0_COMPA_vect) {
  MIL_counter++;                    // increase millis counter
}

// -----------------------------------------------------------------------------
// ADC Implementation
// -----------------------------------------------------------------------------

// Voltage references
#define REFVCC        0                             // Vcc as reference
#define REF1V1        (1<<REFS1)                    // internal 1.1V reference
#define REF2V56       ((1<<REFS2) | (1<<REFS1))     // internal 2.56V reference
#define AVBG          ((1<<MUX3)  | (1<<MUX2))      // port of internal bandgap (1.1V reference)

// ADC init
void ADC_init(void) {
  ADCSRA  = (1<<ADPS2) | (1<<ADPS1);        // set ADC clock prescaler to 64
  ADCSRA |= (1<<ADEN)  | (1<<ADIE);         // enable ADC and ADC interrupt
}

// ADC add up several readings (oversampling) against choosen reference in sleep mode to denoise
// max samples: 64
uint16_t ADC_read(uint8_t port, uint8_t reference, uint8_t samples) {
  uint16_t result = 0;                      // result
  ADCSRA |= (1<<ADEN) | (1<<ADIF);          // enable ADC, turn off any pending interrupt
  ADMUX = (port & 0x0f) | reference;        // set input channel and reference
  if (reference || (port==AVBG)) _delay_ms(2);  // wait 2ms to settle if Vref is used  
  set_sleep_mode (SLEEP_MODE_ADC);          // sleep during sample for noise reduction
  for (; samples; samples--) {              // get several samples
    sleep_mode();                           // go to sleep while taking ADC sample
    while (ADCSRA & (1<<ADSC));             // make sure sampling is completed
    result += ADC;                          // add them up
  }
  return (result);                          // return value
}

// ADC interrupt service routine
EMPTY_INTERRUPT (ADC_vect);                 // nothing to be done here

// -----------------------------------------------------------------------------
// Additional Functions
// -----------------------------------------------------------------------------

// Check if button is pressed
uint8_t buttonPressed(void) {
  uint16_t value = ADC_read(NTC_AP, REFVCC, 1);
  return (value < NTCMIN);
}

// Check if NTC is connected
uint8_t NTCpresent(void) {
  uint16_t value = ADC_read(NTC_AP, REFVCC, 1);
  return ( (value >= NTCMIN) && (value < 1020) );
}

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------

// Text strings stored in program memory
const char StartScreen[] PROGMEM = 
  "-- TinyLoad  v1.1  --"
  "Turn poti full CCW,  "
  "connect the load and "
  "press SET button.    ";
const char WarnScreen[] PROGMEM =
  "!!!!!  WARNING  !!!!!"
  "You reached max power"
  "  or temperature !   "
  "!!! DECREASE LOAD !!!";

// Main function
int main(void) {
  // Local variables
  uint16_t  vcc, vref2, onevolt, twovolt, rawCheck, temp;
  float     rawVoltage, rawCurrent, voltage, current, power, resistance;
  float     capacity = 0;
  float     energy = 0;
  uint8_t   primescreen = 1;
  uint8_t   warnscreen;
  uint8_t   lastbutton;
  uint32_t  lastmillis, nowmillis, interval;

  // Setup
  DDRB |= 1<<FAN_PIN;                                   // set output pins 
  MIL_init();                                           // setup millis counter
  ADC_init();                                           // setup ADC
  OLED_init();                                          // prepare and start OLED
  sei();                                                // enable global interrupts

  // Calibrate internal 2.56V reference
  vref2 = ADC_read(AVBG, REF2V56, 64);                  // read internal bandgap against 2.56V reference
  vref2 = (uint32_t)72019200 / vref2;                   // calculate Vref2 in mV; 72019200 = 64*1023*1100

  // Check if something is connected to the load
  OLED_clearScreen();
  OLED_setCursor(0,0);
  OLED_printPrg(StartScreen);
  while( (!buttonPressed()) && (ADC_read(CURRENT_AP, REF1V1, 1) < 10) );

  // Init some variables;
  lastmillis = MIL_read();
  lastbutton = 1;

  // Loop
  while(1) {
    vcc     = ADC_read(AVBG, REFVCC, 64);               // read internal bandgap against Vcc with 64 samples
    vcc     = (uint32_t)72019200 / vcc;                 // calculate Vcc in mV; 1125300 = 64*1023*1100
    onevolt = (uint32_t)1125300 / vcc;                  // calculate ADC-value for 1.1V
    twovolt = (uint32_t)vref2 * 1023 / vcc;             // calculate ADC-value for 2.56V (calibrated)

    // Read and calculate voltage in V
    rawCheck = ADC_read(VOLTAGE_AP, REFVCC, 1);         // get ADC-value for voltage reading
    if (rawCheck < onevolt) {                           // check possibility of reading against 1.1V reference
      rawVoltage  = ADC_read(VOLTAGE_AP, REF1V1, 64);   // read raw voltage against 1.1V reference
      voltage     = rawVoltage * 1100 / UCONV * ULCAL;  // calculate real voltage value
    }
    else if (rawCheck < twovolt) {                      // check possibility of reading against 2.56V reference
      rawVoltage  = ADC_read(VOLTAGE_AP, REF2V56, 64);  // read raw voltage against 2.56V reference
      voltage     = rawVoltage * vref2 / UCONV * ULCAL; // calculate real voltage value
    }
    else {
      rawVoltage  = ADC_read(VOLTAGE_AP, REFVCC, 64);   // read raw voltage against vcc
      voltage     = rawVoltage * vcc / UCONV * ULCAL;   // calculate real voltage value
    }

    // Read and calculate current in A
    rawCheck = ADC_read(CURRENT_AP, REFVCC, 1);         // get ADC-value for current reading
    if (rawCheck < onevolt) {                           // check possibility of reading against 1.1V reference
      rawCurrent  = ADC_read(CURRENT_AP, REF1V1, 64);   // read raw current against 1.1V reference
      current     = rawCurrent * 1100 / ICONV * ILCAL;  // calculate real current value
    }
    else if (rawCheck < twovolt) {                      // check possibility of reading against 2.56V reference
      rawCurrent  = ADC_read(CURRENT_AP, REF2V56, 64);  // read raw current against 2.56V reference
      current     = rawCurrent * vref2 / ICONV * ILCAL; // calculate real current value
    }
    else {
      rawCurrent  = ADC_read(CURRENT_AP, REFVCC, 64);   // read raw current against vcc
      current     = rawCurrent * vcc / ICONV * ILCAL;   // calculate real current value
    }
  
    // Calculate power and resistance
    power = voltage * current;                          // calculate power in W
    if (current > 0) resistance = voltage / current;    // calculate resistance in Ohm
    else resistance = 9999;                             // 9999 if no current
    if (resistance > 9999) resistance = 9999;           // 9999 is max displayed value

    // Calculate energy/capacity in mWh and mAh
    nowmillis   = MIL_read();
    interval    = nowmillis - lastmillis;               // calculate time interval
    lastmillis  = nowmillis;
    energy   += power   * interval / 3600;              // calculate mWh
    capacity += current * interval / 3600;              // calculate mAh

    // Turn on fan if needed
    temp = ADC_read(NTC_AP, REFVCC, 1);
    if ( (power > 1) || ( (temp >= NTCMIN) && (temp < NTCFAN) ) ) PORTB |= 1<<FAN_PIN;
    else PORTB &= ~(1<<FAN_PIN);

    // Check limits
    warnscreen = ( (power > MAXPOWER) || ( (temp >= NTCMIN) && (temp < NTCHOT) ) );

    // Check button
    if (!lastbutton && buttonPressed()) primescreen = !primescreen;
    lastbutton = buttonPressed();
   
    // Display values on the oled
    OLED_setCursor(0,0);

    if (warnscreen) OLED_printPrg(WarnScreen);
    else {
      OLED_printPrg(PSTR("Voltage:   ")); OLED_printValue(voltage); OLED_printPrg(PSTR(" V"));
      OLED_printPrg(PSTR("Current:   ")); OLED_printValue(current); OLED_printPrg(PSTR(" A"));

      if (primescreen) {
        OLED_printPrg(PSTR("Power:     ")); OLED_printValue(power); OLED_printPrg(PSTR(" W"));
        OLED_printPrg(PSTR("Resistance:")); OLED_printValue(resistance); OLED_printPrg(PSTR(" O"));
      }
      else {
        OLED_printPrg(PSTR("Energy:  ")); OLED_printValue(energy);   OLED_printPrg(PSTR(" mWh"));
        OLED_printPrg(PSTR("Charge:  ")); OLED_printValue(capacity); OLED_printPrg(PSTR(" mAh"));
      }
    }
    _delay_ms(100);
  }
}
