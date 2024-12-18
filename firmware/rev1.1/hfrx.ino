
// ============================================================================
// hfrx.ino :: An experimental SSB receiver
//
// (c) 2024  Scott Baker KJ7NLA
//
// Libraries
// ---------
// ee.h                 - EEPROM library
// i2c0.h               - I2C library
// i2c1.h               - I2C library
// recv.h               - SSB receiver library
// oled.h               - OLED library
// font.h               - OLED font
// lcd.h                - LCD library (optional)
// si5351.h             - si5351 VFO library
//
// Arduino IDE settings
// --------------------
// board: Atmega328 (MiniCore)
// clock: external 20MHz
// BOD: BOD 2.7V
// eeprom: eeprom retained
// compiler: LTO enabled
// variant: 328PB
// pinout: standard
// bootloader: no bootloader
// programmer: AVRISP mkII
//
// Acknowledgment
// --------------
// This work is partly based on the uSDX architecture by PE1NNZ
// and the receiver code is from https://github.com/threeme3/usdx-sketch
// Copyright 2019, 2020, 2021, 2022, 2023, 2024
// Guido PE1NNZ <pe1nnz@amsat.org>
// and is used by permission from the author
//
// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// ============================================================================

#define VERSION   "hfrx 1.0A"           // firmware version
#define DATE      "Dec 2 2024"          // firmware date

#include "globals.h"
#include "ee.h"
#include "i2c0.h"
#include "i2c1.h"
#include "recv.h"
#include "oled.h"
#include "font.h"
#include "si5351.h"

// prototype defs
char getc();
char gnac();
void getsemi();
char gcal(char ch);
uint8_t len(char *str);
void send(char *str);
uint8_t cmpstr(char *dst, char *src);
void catstr(char *dst, char c);
void uppercase(char *str);
uint8_t alpha(char ch);
uint8_t numeric(char ch);
uint32_t fs2int(char *str);
void cpy(char *dst, char *src);
void cat(char *dst, char *src);
void show_version();
void show_help();
void show_fxtal();
void show_cal(int16_t step);
void show_info();
void show_debug();
void print_version();
void update_display();
char* int2str(uint8_t val);
char* freq2str(uint32_t val);
void wait_ms(uint16_t dly);
void wait_us(uint16_t x);
void blinkLED();
void stepsize_cursor();
void CAT_VFO();
void CAT_cmd();
void reset_xtimer();
void check_timeout();
void check_UI();
void check_menu();
void exit_menu();
void update_freq(uint8_t x);
void lock_encoder();
void menuAction(uint8_t id);
void paramAction(uint8_t id, uint8_t* ptr, const char* sap[], uint8_t min, uint8_t max);
void get_label(uint8_t id, const char* str);
void show_label(uint8_t id);
void do_reset(uint8_t soft);
void calibrate();
void save2ee();
void menu_reset();
void save_eeprom();
void init_soft();
void init_factory();
void init_timer0();
void init_pins();
void init_i2c();
void init_oled();
void init_vfo();
void init_uart();
void init_encoder();
void init_recv();

// eeprom addresses
#define CAL_ADDR    10       // vfo cal data
#define FREQ_ADDR   20       // vfo freq
#define STEP_ADDR   30       // vfo step size
#define VOL_ADDR    31       // volume
#define MODE_ADDR   32       // mode (USB/LSB/CW)
#define BAND_ADDR   33       // band
#define FILT_ADDR   34       // band
#define RXAT_ADDR   35       // analog  attenuation
#define DIGA_ADDR   36       // digital attenuation
#define AGC_ADDR    37       // agc
#define TONE_ADDR   38       // CW tone
#define DXBK_ADDR   39       // display blanking (on/off)

// class instantiation
EE      eeprom;
I2C0    i2c0;
I2C1    i2c1;
RECV    recv;
OLED    oled;
SI5351  si5351;

#define F_XTAL  27000000UL
#define F_CPU   20000000UL

// radio bands
#define BAND_80M  0
#define BAND_60M  1
#define BAND_40M  2
#define BAND_30M  3
#define BAND_20M  4
#define BAND_17M  5
#define BAND_15M  6
#define BAND_12M  7
#define BAND_10M  8

// menu states
#define NOT_IN_MENU   0
#define SELECT_MENU   1
#define SELECT_VALUE  2

// delay times (ms)
#define DEBOUNCE           50
#define LED_BLINK         100
#define ERROR_BLINK       200
#define HALF_SECOND       500
#define ONE_SECOND       1000
#define TWO_SECONDS      2000
#define THREE_SECONDS    3000
#define TEN_SECONDS     10000
#define HALF_MINUTE     30000
#define ONE_MINUTE      60000
#define TWO_MINUTES    120000
#define FIVE_MINUTES   600000
#define HALF_HOUR     3600000

// user interface buttons
#define NBP          0  // no-button-pushed
#define CLICK        1  // button-single-click
#define LONG         2  // button-push-long
#define LONGPRESS  600  // ms

// read pushbutton switches
#define SW1_PRESSED  !digitalRead(SW1)
#define SW2_PRESSED  !digitalRead(SW2)
#define SW3_PRESSED  !digitalRead(SW3)
#define ANY_PRESSED  SW1_PRESSED | SW2_PRESSED

// CW tones
#define T600      0
#define T700      1

uint16_t ct[] = {600, 700};

// step sizes
#define STEP_0    0
#define STEP_1    1
#define STEP_10   2
#define STEP_100  3
#define STEP_1K   4
#define STEP_10K  5
#define STEP_100K 6
#define STEP_1M   7

uint32_t stepsizes[] = { 0, 1, 10, 100, 1000, 10000, 100000, 1000000 };

#define INIT_FREQ  14100000UL

// menu id values
#define VOLUME      0
#define RADIOMODE   1
#define RADIOBAND   2
#define FILTERBW    3
#define RX_ATTN     4
#define DG_ATTN     5
#define AGC         6
#define CWTONE      7
#define DXBLANK     8
#define CALIBRATE   9
#define SAVE2EE     10
#define RESET       11
#define SWVER       12

#define FIRSTMENU  VOLUME
#define LASTMENU   SWVER

// globals
uint8_t  DEBUG = FALSE;
int32_t  vfofreq  = INIT_FREQ;
int32_t  catfreq  = vfofreq;
uint8_t  menumode = NOT_IN_MENU;
uint8_t  event = NBP;
uint8_t  enc_locked = NO;
int8_t   menu = VOLUME;

// used by recv module
volatile uint8_t rxstate = 0;

// for display blank/timeout
uint8_t  display = ON;
uint32_t xtimer;

// menu labels
const char mlabel[] = "\
Volume|Radio Mode|Radio Band|Filter|Rx Attn|Dig Attn|\
AGC|CW Tone|OLED Timeout|Calibrate|Save to EE|Factory Reset|Version|";

// menu variables
uint8_t  stepsize   = STEP_1K;   // freq tuning step size
uint8_t  radiomode  = USB;       // radio mode
uint8_t  radioband  = BAND_20M;  // radio band
uint8_t  prevband   = BAND_20M;  // previous radio band
uint8_t  filterbw   = BWFULL;    // filter bandwidth
uint8_t  iq_phase   = 90;        // quadrature phase
uint8_t  volume     = 10;        // audio volume
uint8_t  rx_attn    = 0;         // analog  attenuation
uint8_t  dg_attn    = 4;         // digital attenuation
uint8_t  agc        = OFF;       // auto gain control
uint8_t  cwtone     = T600;      // CW tone select
uint8_t  dxblank    = ON;        // display blanking

const char* band_label[]   = { "80M", "60M", "40M", "30M", "20M", "17M", "15M", "12M", "10M" };
const char* mode_label[]   = { "USB", "LSB", "CW"};
const char* filtbw_label[] = { "1500", "2000", "2500", "FULL" };
const char* cwtone_label[] = { "600", "700" };
const char* dxbk_label[]   = { "OFF", "5 Minutes", "30 Minutes"};
const char* onoff_label[]  = { "OFF", "ON" };
const char* rxatt_label[]  = { "OFF", "-6dB" };
const char* dgatt_label[]  = { "-36dB", "-30dB", "-24dB", "-18dB", "-12dB", "-6dB", "OFF" };

// millisecond time
volatile uint32_t msTimer = 0;

// timer 0 interrupt service routine
ISR(TIMER0_COMPA_vect) {
  msTimer++;
}

// rotary encoder variables
volatile int8_t  enc_val  = 0;
volatile int8_t  enc_step = 0;
volatile uint8_t enc_state;
volatile uint8_t enc_a;
volatile uint8_t enc_b;

inline void enc() {
  enc_state = (enc_state << 4) | (enc_b << 1) | enc_a;
  switch (enc_state) {
    case 0x23:  enc_val++; break;
    case 0x32:  enc_val--; break;
    default: break;
  }
  if (enc_locked || !display) enc_val = 0;
  reset_xtimer();
}

// rotary encoder interrupt handler (A)
ISR(PCINT2_vect) {
  enc_a = digitalRead(ROTA);
  enc();
}

// rotary encoder interrupt handler (B)
ISR(PCINT0_vect) {
  enc_b = digitalRead(ROTB);
  enc();
}

// ADC sample clock interrupt
ISR(TIMER2_COMPA_vect) {
  recv.sample_dsp();
}

// read char from the serial port
char getc() {
  while (!Serial.available());
  return(Serial.read());
}

// get next alpha char from serial buffer
char gnac() {
  char ch;
  uint16_t tc = 0;  // loop counter
  while (TRUE) {
    if (Serial.available()) {
      ch = Serial.read();
      if (alpha(ch)) return(ch);
    } else {
      if (tc++ > 1024) return('z');
    }
  }
}

// wait for semicolon from serial buffer
void getsemi() {
  char ch;
  while (TRUE) {
    if (Serial.available()) {
      ch = Serial.read();
      if (ch == ';') return;
    }
  }
}

// get cal control char from serial buffer
char gcal(char ch) {
  char new_ch;
  uint16_t tc = 0;  // loop counter
  while (TRUE) {
    if (Serial.available()) {
      new_ch = Serial.read();
      if ((new_ch=='+')||(new_ch=='-')||
          (new_ch=='/')||(new_ch=='\\')||
          (new_ch=='=')||(new_ch=='.')){
        return(new_ch);
      }
    } else {
      if (tc++ > 1024) return(ch);
    }
  }
}

// return the length of string
uint8_t len(char *str) {
  uint8_t i=0;
  while (str[i++]);
  return i-1;
}

// send a command
void send(char *str) {
  getsemi(); // get semicolon
  Serial.print(str);
}

// compare command
uint8_t cmpstr(char *x, char *y) {
  if ((x[0] == y[0]) && (x[1] == y[1])) return(1);
  else return(0);
}

// concatenate a character to a string
void catstr(char *dst, char ch) {
  uint8_t strlen= len(dst);
  dst[strlen++] = ch;
  dst[strlen] = '\0';
}

// convert command to upper case
void uppercase(char *str) {
  if ((str[0] >= 'a') && (str[0] <= 'z')) str[0] -= 32;
  if ((str[1] >= 'a') && (str[1] <= 'z')) str[1] -= 32;
}

// check if char is alpha
uint8_t alpha(char ch) {
  if ((ch >= 'A') && (ch <= 'z')) return(1);
  return(0);
}

// check if char is numeric
uint8_t numeric(char ch) {
  if ((ch >= '0') && (ch <= '9')) return(1);
  return(0);
}

// convert frequency string to integer
uint32_t fs2int(char *str) {
  uint32_t acc = 0;
  uint32_t pwr10 = 1;
  uint8_t digit;
  for (uint8_t i=10; i>2; i--) {
    digit = (uint8_t)str[i] - 48;
    acc += digit * pwr10;
    pwr10 = ((pwr10<<3)+(pwr10<<1));
  }
  return(acc);
}

// copy a string
void cpy(char *dst, char *src) {
  uint8_t i=0;
  while (src[i]) dst[i++] = src[i];
  dst[i] = '\0';
}

// concatenate a string
void cat(char *dst, char *src) {
  uint8_t i=0;
  uint8_t strlen = len(dst);
  while (src[i]) dst[strlen++] = src[i++];
  dst[strlen] = '\0';
}

// display the firmware version
void show_version() {
  oled.clrScreen();
  oled.printline(0, "SSB Receiver");
  oled.printline(1, VERSION);
  print_version();
  wait_ms(TWO_SECONDS);
  update_display();
}

#define HELP_MSG "\r\n\
  IF  G -  radio status\r\n\
  ID  G -  radio ID\r\n\
  FA  G S  frequency\r\n\
  AI  G S  auto-information\r\n\
  MD  G S  radio mode\r\n\
  PS  G S  power-on status\r\n\
  XT  G S  XIT status\r\n\
  TX  - S  transmit\r\n\
  RX  - S  receive\r\n\n\
  HE => print help\r\n\
  HH => print help\r\n\
  DD => debug on/off\r\n\
  II => print info\r\n\
  FR => factory reset\r\n\
  SR => soft reset\r\n\n"

// print help message
void show_help() {
  Serial.print(HELP_MSG);
}

// print xtal calibration value
void show_fxtal() {
  oled.clrLine(1);
  oled.print32(si5351.fxtal);
}

// print calibration step
void show_cal(int16_t step) {
  oled.clrLine(0);
  oled.putstr("Cal step=");
  oled.setCursor(11, 0);
  oled.print16(step);
}

// print info to serial port
void show_info() {
  print_version();
  // print band
  Serial.print("band = ");
  Serial.println(band_label[radioband]);
  // print frequency
  Serial.print("freq = ");
  Serial.print(vfofreq);
  Serial.print("\r\n");
  // print mode
  Serial.print("mode = ");
  Serial.println(mode_label[radiomode]);
}

// show debug status
void show_debug() {
  DEBUG = ! DEBUG;
  Serial.print("DEBUG=");
  Serial.print(DEBUG);
  Serial.println("");
}

// print the firmware version to serial port
void print_version() {
  Serial.println('\n');
  Serial.println(VERSION);
  Serial.println(DATE);
}

// update display with mode/band and vfo frequency
void update_display() {
  oled.clrScreen();
  char tmp[20];
  cpy(tmp, mode_label[radiomode]);
  cat(tmp, "   ");
  cat(tmp, band_label[radioband]);
  oled.printline(0, tmp);
  update_freq(0);
}

// convert 8-bit integer to string
char* int2str(uint8_t val) {
  static char tmp[4];
  uint8_t i;
  for (i = 0; i < 3; i++) tmp[i] = ' ';
  if (!val) {
    tmp[2] = '0';
    return (tmp);
  }
  for (i = 2; val; i--) {
    tmp[i] = "0123456789"[val % 10];
    val /= 10;
  }
  return (tmp);
}

// convert 32-bit integer to string
char* freq2str(uint32_t val) {
  static char str[11];
  for (uint8_t i = 0; i < 10; i++) str[i] = ' ';
  for (uint8_t i = 9; val; i--) {
    if ((i == 6) || (i == 2)) {
      str[i] = ',';
      i--;
    }
    str[i] = "0123456789"[val % 10];
    val /= 10;
  }
  str[10] = '\0';
  return (str);
}

// millisecond delay
void wait_ms(uint16_t dly) {
  uint32_t startTime = msTimer;
  while((msTimer - startTime) < dly) {
    wait_us(10);
  }
  if (dly == DEBOUNCE) reset_xtimer();
}

// microsecond delay
void wait_us(uint16_t x) {
  uint16_t t = ((x * 3) + (x>>1));
  for (int16_t i=0; i<t; i++) {
    asm("");
  }
}

// blink the LED
void blinkLED() {
  digitalWrite(XLED, ON);
  wait_ms(LED_BLINK);
  digitalWrite(XLED, OFF);
}

// set the location of the stepsize cursor
void stepsize_cursor() {
  switch (stepsize) {
    case STEP_1:    oled.setCursor(9,1); break;
    case STEP_10:   oled.setCursor(8,1); break;
    case STEP_100:  oled.setCursor(7,1); break;
    case STEP_1K:   oled.setCursor(5,1); break;
    case STEP_10K:  oled.setCursor(4,1); break;
    case STEP_100K: oled.setCursor(3,1); break;
    case STEP_1M:   oled.setCursor(1,1); break;
    default:        oled.setCursor(9,1); break;
  }
  oled.showCursor(ON);
}

// ==============================================================
// The following Kenwood TS-2000 CAT commands are implemented
//
// command get/set  name              operation
// ------- -------  ----------------  -----------------------
// IF        G -    radio status      returns frequency and other status
// ID        G -    radio ID          returns 019 = Kenwood TS-2000
// FA        G S    frequency         gets or sets the ADX frequency
// AI        G S    auto-information  returns 0   = OFF
// MD        G S    radio mode        returns 2   = USB
// PS        G S    power-on status   returns 1   = ON
// XT        G S    XIT status        returns 0   = OFF
// TX        - S    transmit          returns 0 and set TX LED
// RX        - S    receive           returns 0 and clears TX LED
//
// The following ADX-specific CAT commands are implemented
//
//  HE => print help
//  HH => print help
//  II => print info
//  DD => turn on/off debug
//  FR => factory reset
//  SR => soft reset
//  CM => calibration mode
// ==============================================================

// check for CAT control
void check_CAT() {
  if (Serial.available()) CAT_cmd();
  if (vfofreq != catfreq) {
    vfofreq = catfreq;
    update_display();
  }
}

// print (11-bit) VFO frequency
void CAT_VFO() {
  if      (vfofreq >= 10000000) Serial.print("000");
  else if (vfofreq >=  1000000) Serial.print("0000");
  else                          Serial.print("00000");
  Serial.print(vfofreq);
}

void CAT_cmd() {
  char cmd[3] = "zz";
  char param[20] = "";

  // get next alpha char from serial buffer
  char ch = gnac();
  if (ch == 'z') return;  // non-alpha char

  // get the command
  cmd[0] = ch;
  cmd[1] = getc();
  uppercase(cmd);

  // ===========================
  //  TS-2000 CAT commands
  // ===========================

  //====================================
  //  IF           // (command)       2
  //  00014074000  // P1 (VF0)       11
  //  0000         // P2 (step size)  4
  //  +00000       // P3 (rit)        6
  //  00000        // P4->P7          5
  //  0/1          // P8 (Tx/Rx)      1
  //  20000000     // P9->P15         8
  //                       TOTAL  =  37
  //====================================

  // get frequency and other status
  if (cmpstr(cmd, "IF")) {
    send("IF");
    CAT_VFO();
    Serial.print("00000+000000000");
    Serial.print('0'); // always rx
    Serial.print("20000000;");
  }

  // get radio ID
  else if (cmpstr(cmd, "ID")) send("ID019;");

  // get or set frequency
  else if (cmpstr(cmd, "FA")) {
    ch = getc();
    if (numeric(ch)) {
      // set frequency
      catstr(param, ch);
      for (uint8_t i=0; i<10; i++) {
        catstr(param, getc());
      }
      catfreq = fs2int(param);
      getsemi(); // get semicolon
    } else {
      // get frequency
      Serial.print("FA");
      CAT_VFO();
      Serial.print(";");
    }
  }

  // get or set the radio mode
  else if (cmpstr(cmd, "MD")) {
    ch = getc();
    if (numeric(ch)) {
      // set radio mode
      // does nothing .. always 2
      getsemi();
    } else {
      // get auto-information status
      Serial.print("MD2;");
    }
  }

  // get or set auto-information status
  else if (cmpstr(cmd, "AI")) {
    ch = getc();
    if (numeric(ch)) {
      // set auto-information status
      // does nothing .. always 0
      getsemi();
    } else {
      // get auto-information status
      Serial.print("AI0;");
    }
  }

  // get or set the power (ON/OFF) status
  else if (cmpstr(cmd, "PS")) {
    ch = getc();
    if (numeric(ch)) {
      // set power (ON/OFF) status
      // does nothing .. always 1
      getsemi();
    } else {
      // get power (ON/OFF) status
      Serial.print("PS1;");
    }
  }

  // get or set the XIT (ON/OFF) status
  else if (cmpstr(cmd, "XT")) {
    ch = getc();
    if (numeric(ch)) {
      // set XIT (ON/OFF) status
      // does nothing .. always OFF
      getsemi();
    } else {
      // get XIT (ON/OFF) status
      Serial.print("XT0;");
    }
  }

  // CAT transmit -- always rx
  else if (cmpstr(cmd, "TX")) {
    getsemi(); // get semicolon
  }

  // CAT receive -- always rx
  else if (cmpstr(cmd, "RX")) {
    getsemi(); // get semicolon
  }

  // ===========================
  //  ADX-specific CAT commands
  // ===========================

  // print help
  if (cmpstr(cmd, "HE")) {
    show_help();
  }

  // print help
  else if (cmpstr(cmd, "HH")) {
    show_help();
  }

  // toggle debug on/off
  else if (cmpstr(cmd, "DD")) {
    show_debug();
  }

  // print info
  else if (cmpstr(cmd, "II")) {
    show_info();
  }

  // factory reset
  else if (cmpstr(cmd, "FR")) {
    do_reset(FACTORY);
  }

  // soft reset
  else if (cmpstr(cmd, "SR")) {
    do_reset(SOFT);
  }
}

// reset display timeout
void reset_xtimer() {
  xtimer = msTimer;
  if (display == OFF) {
    display = ON;
    oled.onDisplay();
  }
}

// display timeout
uint32_t display_timeout = FIVE_MINUTES;

// check for display timeout
void check_timeout() {
  if (!dxblank) return;  // skip if disabled
  if ((display == ON) && ((msTimer - xtimer) > display_timeout)) {
    display = OFF;
    oled.noDisplay();
  }
}

// check the UI pushbuttons
void check_UI() {
  uint8_t event = NBP;
  uint32_t t0 = msTimer;
  if (display == OFF) {
    // check for wake-up
    while (SW1_PRESSED) wait_ms(DEBOUNCE);
    while (SW2_PRESSED) wait_ms(DEBOUNCE);
    while (SW3_PRESSED) wait_ms(DEBOUNCE);
    return;
  }
  if (SW1_PRESSED) {
    // SW1 button click controls the menu system
    // SW1 button has no long-press function
    switch (menumode) {
      case NOT_IN_MENU:
        // enter menu selection mode
        menumode = SELECT_MENU;
        oled.clrScreen();
        oled.showCursor(OFF);
        enc_locked = NO;
        enc_val = 0;
        break;
      case SELECT_MENU:
        // enter value selection mode
        menumode = SELECT_VALUE;
        break;
      case SELECT_VALUE:
        // enter menu selection mode
        menumode = SELECT_MENU;
        break;
      default:
        menumode = NOT_IN_MENU;
        break;
    }
    menuAction(menu);
    // wait for SW1 released
    while (SW1_PRESSED) wait_ms(DEBOUNCE);
  } else if (SW2_PRESSED) {
    // SW2 button click when in menu mode will exit the menu
    // SW2 button click when not in menu mode will update the step size
    // SW2 button long press changes bands with encoder
    event = CLICK;
    if (menumode) {
      exit_menu();
      // wait for SW2 released
      while (SW2_PRESSED) wait_ms(DEBOUNCE);
    } else {
      while (SW2_PRESSED) {
        // check for long press
        if ((msTimer - t0) > LONGPRESS) { event = LONG; break; }
        wait_ms(DEBOUNCE);
      }
      if (event == LONG) {
        // during an SW2 long press
        // use the encoder to update the radio band
        oled.clrScreen();
        menumode = SELECT_VALUE;
        menuAction(RADIOBAND);
        while (SW2_PRESSED) {
          if (enc_val) menuAction(RADIOBAND);
          wait_ms(DEBOUNCE);
        }
      } else {
        // SW2 click updates the step size
        stepsize_cursor();
        stepsize--;
        if (stepsize < STEP_1)  stepsize = STEP_1M;
        stepsize_cursor();
        wait_ms(DEBOUNCE);
      }
      exit_menu();
    }
  } else if (SW3_PRESSED) {
    // SW3 button click locks/unlocks the encoder
    // SW3 button long press controls volume with encoder
    if (!menumode) {
      event = CLICK;
      while (SW3_PRESSED) {
        // check for long press
        if ((msTimer - t0) > LONGPRESS) { event = LONG; break; }
        wait_ms(DEBOUNCE);
      }
      if (event == LONG) {
        // during an SW3 long press
        // use the encoder to update the volume
        oled.clrScreen();
        menumode = SELECT_VALUE;
        menuAction(VOLUME);
        enc_locked = NO;
        while (SW3_PRESSED) {
          if (enc_val) menuAction(VOLUME);
          wait_ms(DEBOUNCE);
        }
      } else {
        // toggle encoder lock/unlock
        lock_encoder();
      }
      exit_menu();
    }
  } else {
    // no buttons are pressed
    // use the encoder to update the VFO
    if (enc_val && !menumode) {
      update_freq(1);
    }
    check_timeout();   // check for display timeout
  }
}

// check the menu state
void check_menu() {
  if (menumode == SELECT_MENU) {
    if (enc_val) {
      menu += enc_val;
      // check menu for lower and upper limits
      // and wrap around at limits
      if (menu < FIRSTMENU) menu = LASTMENU;
      else if (menu > LASTMENU) menu = FIRSTMENU;
      enc_val = 0;
      menuAction(menu);
    }
  }
  if (menumode == SELECT_VALUE) {
    if (enc_val) {
      menuAction(menu);
    }
  }
}

const int32_t bandfreq[] = {
  3573000,  5357000,  7074000,  10136000, 14074000,
  18100000, 21074000, 24915000, 28074000
};

// exit menu and update display
void exit_menu() {
  menumode = NOT_IN_MENU;
  enc_val = 0;
  // check if band has changed
  if (radioband != prevband) {
    prevband = radioband;
    vfofreq = bandfreq[radioband];
    catfreq = vfofreq;
  }
  update_display();
}

void lock_encoder() {
  enc_locked = !enc_locked;
  oled.clrScreen();
  oled.showCursor(OFF);
  oled.printline(0, "Encoder");
  if (enc_locked) oled.printline(1, "Locked");
  else oled.printline(1, "Unlocked");
  wait_ms(ONE_SECOND);
}


// *** START OF MENU SUBROUTINES

// menu actions
void menuAction(uint8_t id) {

  switch (id) {
    //   id                          variable    label         min max
    //   ---------                   --------    -------       --- ---
    case VOLUME:     paramAction(id, &volume,    NULL,         5, 12); break;
    case RADIOMODE:  paramAction(id, &radiomode, mode_label,   0,  2); break;
    case RADIOBAND:  paramAction(id, &radioband, band_label,   0,  8); break;
    case FILTERBW:   paramAction(id, &filterbw,  filtbw_label, 0,  3); break;
    case RX_ATTN:    paramAction(id, &rx_attn,   rxatt_label,  0,  1); break;
    case DG_ATTN:    paramAction(id, &dg_attn,   dgatt_label,  0,  6); break;
    case AGC:        paramAction(id, &agc,       onoff_label,  0,  1); break;
    case CWTONE:     paramAction(id, &cwtone,    cwtone_label, 0,  1); break;
    case DXBLANK:    paramAction(id, &dxblank,   dxbk_label,   0,  2); break;
    case CALIBRATE:  calibrate(); break;
    case SAVE2EE:    save2ee(); break;
    case RESET:      menu_reset(); break;
    case SWVER:      paramAction(id, NULL,       NULL,         0,  1); break;

    default:      break;
  }
}

// parameters actions
void paramAction(uint8_t id, uint8_t* ptr, const char* sap[], uint8_t min, uint8_t max) {
  uint8_t value = *ptr;
  int16_t newvalue;
  switch (menumode) {
    case SELECT_MENU:
      show_label(id);              // show menu label
      show_value(id, value, sap);  // show menu value
      break;
    case SELECT_VALUE:
      show_label(id);              // show menu label
      // read encoder and update value
      newvalue = value + enc_val;
      // check min and max value limits
      if (newvalue < min) value = min;
      else if (newvalue > max) value = max;
      else value = newvalue;
      // parameter-specific actions
      switch (id) {
        case SWVER:
          menumode = SELECT_MENU;
          value = 0;
          break;
        case DXBLANK:
          switch (value) {
            case 1:
              display_timeout = FIVE_MINUTES;
              break;
            case 2:
              display_timeout = HALF_HOUR;
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
      enc_val = 0;
      *ptr = value;
      show_value(id, value, sap);
      break;
    default:
      break;
  }
}

char menulabel[20];

// get label string
void get_label(uint8_t id, const char* str) {
  uint8_t  done = 0;
  uint16_t i = 0;
  uint8_t  j = 0;
  uint8_t  k = 0;
  char ch = ' ';
  while (!done) {
    ch = str[i++];
    if (ch != '|') {
      menulabel[j++] = ch;
    } else {
      if ((menumode == SELECT_VALUE) && (menu != SWVER)) {
        menulabel[j++] = ' ';
        menulabel[j++] = '>';
      }
      menulabel[j] = 0;
      if (k == id) done = 1;
      else j = 0;
      k++;
    }
    if (!ch) done = 1;
  }
}

// print a menu label
void show_label(uint8_t id) {
  get_label(id, mlabel);
  oled.clrLine(0);
  oled.printline(0, menulabel);
}

// print a menu value field
void show_value (uint8_t id, uint8_t val, const char* sap[]) {
  oled.clrLine(1);
  switch (id) {
    case SWVER:
      oled.printline(1, VERSION);
      break;
    default:
      if (sap == NULL) {
        oled.printline(1, int2str(val));
      } else {
        oled.printline(1, (char *)sap[val]);
      }
  }
}

// update the displayed frequency
void update_freq(uint8_t x) {
  int32_t stepval = stepsizes[stepsize];
  if (x) vfofreq += enc_val * stepval;
  if (radiomode == CW) si5351.freq(vfofreq - ct[cwtone], 0, iq_phase);
  else si5351.freq(vfofreq, 0, iq_phase);
  enc_val = 0;
  oled.printline(1, freq2str(vfofreq));
  catfreq = vfofreq;
  stepsize_cursor();
}

// reset (CAT command)
void do_reset(uint8_t soft) {
  reset_xtimer();
  oled.clrScreen();
  if (soft) {
    // soft reset
    oled.putstr("SOFT RESET");
    Serial.print("Soft Reset\r\n");
    init_soft();
  } else {
    // factory reset
    oled.putstr("FACTORY RESET");
    Serial.print("Factory Reset\r\n");
    init_factory();
  }
}

// calibrate
void calibrate() {
  uint8_t  save = 1;
  int16_t  step = 1;
  int16_t  eval;
  show_label(CALIBRATE);  // show menu label
  switch (menumode) {
    case SELECT_MENU:
      oled.clrLine(1);
      break;
    case SELECT_VALUE:
      show_cal(step);
      show_fxtal();
      // stay in cal mode until SW2 is pressed
      while (!SW2_PRESSED) {
        // zero the frequency adjustment
        if (SW1_PRESSED) {
          switch (step) {
            case 1:
              step = 10;
              break;
            case 10:
              step = 100;
              break;
            case 100:
              step = 1000;
              break;
            case 1000:
              step = 1;
              break;
            default:
              step = 1;
              break;
          }
          show_cal(step);
          // wait for SW1 released
          while (SW1_PRESSED) wait_ms(DEBOUNCE);
        }
        // adjust the frequency
        if (enc_val) {
          if (enc_val > 0) eval = step;
          else eval = -step;
          si5351.fxtal -= eval;
          si5351.fxadj += eval;
          show_fxtal();
          enc_val = 0;
          si5351.freq(vfofreq, 0, iq_phase);
        }
      }
      // wait for SW2 released
      while (SW2_PRESSED) wait_ms(DEBOUNCE);
      oled.clrScreen();
      oled.printline(0, "Save Calibration?");
      oled.printline(1, "YES");

      // stay in yes/no loop until SW2 is pressed
      while (!SW2_PRESSED) {
        if (enc_val) {
          save = !save;
          enc_val = 0;
          oled.clrLine(1);
          if (save) oled.printline(1, "YES");
          else oled.printline(1, "NO");
        }
      }
      oled.clrScreen();
      oled.printline(0, "Calibration");
      if (save) {
        eeprom.put32(CAL_ADDR, si5351.fxadj);
        oled.printline(1, "saved to eeprom");
      } else {
        oled.printline(1, "not saved");
      }
      wait_ms(TWO_SECONDS);
      exit_menu();
      // wait for SW2 released
      while (SW2_PRESSED) wait_ms(DEBOUNCE);
      break;
    default:
      break;
  }
}

// write config data to the eeprom
void save2ee() {
  show_label(SAVE2EE);
  switch (menumode) {
    case SELECT_MENU:
      oled.clrLine(1);
      break;
    case SELECT_VALUE:
      // stay here until SW2 is pressed
      while (!SW2_PRESSED) {
        // save to eeprom if SW1 pressed
        if (SW1_PRESSED) {
          oled.printline(1, "saving eeprom");
          save_eeprom();
          wait_ms(ONE_SECOND);
          // wait for SW1 released
          while (SW1_PRESSED) wait_ms(DEBOUNCE);
          break;
        }
      }
      // wait for SW2 released
      while (SW2_PRESSED) wait_ms(DEBOUNCE);
      exit_menu();
      break;
    default:
      break;
  }
}

// factory reset from menu
void menu_reset() {
  show_label(RESET);
  switch (menumode) {
    case SELECT_MENU:
      oled.clrLine(1);
      break;
    case SELECT_VALUE:
      // stay here until SW2 is pressed
      while (!SW2_PRESSED) {
        // save to eeprom if SW1 pressed
        if (SW1_PRESSED) {
          do_reset(FACTORY);
          wait_ms(ONE_SECOND);
          // wait for SW1 released
          while (SW1_PRESSED) wait_ms(DEBOUNCE);
          break;
        }
      }
      // wait for SW2 released
      while (SW2_PRESSED) wait_ms(DEBOUNCE);
      exit_menu();
      break;
    default:
      break;
  }
}

// write config data to the eeprom
void save_eeprom() {

/*
  Serial.println(vfofreq);
  Serial.println(stepsize);
  Serial.println(volume);
  Serial.println(radiomode);
  Serial.println(radioband);
  Serial.println(filterbw);
  Serial.println(rx_attn);
  Serial.println(dg_attn);
  Serial.println(agc);
  Serial.println("\n\n");
*/

  eeprom.put32(FREQ_ADDR, vfofreq);
  eeprom.put(STEP_ADDR, stepsize);
  eeprom.put(VOL_ADDR,  volume);
  eeprom.put(MODE_ADDR, radiomode);
  eeprom.put(BAND_ADDR, radioband);
  eeprom.put(FILT_ADDR, filterbw);
  eeprom.put(RXAT_ADDR, rx_attn);
  eeprom.put(DIGA_ADDR, dg_attn);
  eeprom.put(AGC_ADDR,  agc);
  eeprom.put(TONE_ADDR, cwtone);
  eeprom.put(DXBK_ADDR, dxblank);
}

// read config data from the eeprom
void init_soft() {
  vfofreq = eeprom.get32(FREQ_ADDR);
  stepsize  = eeprom.get(STEP_ADDR);
  volume    = eeprom.get(VOL_ADDR);
  radiomode = eeprom.get(MODE_ADDR);
  radioband = eeprom.get(BAND_ADDR);
  filterbw  = eeprom.get(FILT_ADDR);
  rx_attn   = eeprom.get(RXAT_ADDR);
  dg_attn   = eeprom.get(DIGA_ADDR);
  agc       = eeprom.get(AGC_ADDR);
  cwtone    = eeprom.get(TONE_ADDR);
  dxblank   = eeprom.get(DXBK_ADDR);

/*
  Serial.println(vfofreq);
  Serial.println(stepsize);
  Serial.println(volume);
  Serial.println(radiomode);
  Serial.println(radioband);
  Serial.println(filterbw);
  Serial.println(rx_attn);
  Serial.println(dg_attn);
  Serial.println(agc);
  Serial.println("\n\n");
*/

}

// factory init values
void init_factory() {
  DEBUG = FALSE;
  menumode = NOT_IN_MENU;
  event = NBP;
  enc_locked = NO;
  menu = VOLUME;
  si5351.fxadj = 0;
  si5351.fxtal = F_XTAL;
  catfreq  = INIT_FREQ;
  vfofreq  = INIT_FREQ;
  stepsize  = STEP_1K;   // freq tuning step size
  volume    = 10;        // audio volume
  radiomode = USB;       // radio mode
  radioband = BAND_20M;  // radio band
  filterbw  = BWFULL;    // filter bandwidth
  rx_attn   = 0;         // analog  attenuation
  dg_attn   = 4;         // digital attenuation
  agc       = OFF;       // auto gain control
  save_eeprom();
}

// *** END OF MENU SUBROUTINES

// initialize timer 0
void init_timer0() {
  TCCR0A = 0x02;          // CTC mode
  OCR0A  = 249;           // 1 ms count value
  TCCR0B = 0x03;          // use clk/64
  TIMSK0 = 0x02;          // interrupt on
}

// init pins
void init_pins() {
  digitalWrite(XLED, OFF);
  digitalWrite(TONE, OFF);
  pinMode(XLED,  OUTPUT);
  pinMode(TONE,  OUTPUT);
  pinMode(QSDI,  INPUT);
  pinMode(QSDQ,  INPUT);
  pinMode(SW1,   INPUT_PULLUP);
  pinMode(SW2,   INPUT_PULLUP);
  pinMode(SW3,   INPUT_PULLUP);
  pinMode(ROTA,  INPUT_PULLUP);
  pinMode(ROTB,  INPUT_PULLUP);
}

// init i2c bus
void init_i2c() {
  i2c0.begin();
  i2c1.begin();
}

// init display
void init_oled() {
  oled.begin();
  display = ON;
  oled.onDisplay();
}

// init vfo
void init_vfo() {
  int32_t tmp;
  tmp = eeprom.get32(CAL_ADDR);
  // check if already calibrated
  if ((tmp > 6000) || (tmp < -6000)) {
    // use the factory defaults
    si5351.fxadj = 0;
    si5351.fxtal = F_XTAL;
  } else {
    // use the saved eeprom value
    si5351.fxadj = tmp;
    si5351.fxtal = F_XTAL - tmp;
  }
  si5351.iqmsa = 0;   // PLL reset
}

#define BAUDRATE 115200

// init serial/debug port
void init_uart() {
  Serial.begin(BAUDRATE);
  UCSR0A = 0x02;  // set the u2x bit
  // UBRR = (F_CPU / baud) / 8;
  UBRR0H = 0x00;  // baud (upper-byte)
  UBRR0L = 0x15;  // baud (lower-byte)
}

// rotary encoder init
void init_encoder() {
  // interrupt-enable for ROTA, ROTB pin changes
  PCMSK0 = (1 << PCINT0);
  PCMSK2 = (1 << PCINT23);
  PCICR  = (1 << PCIE0) | (1 << PCIE2);
  enc_a = digitalRead(ROTA);
  enc_b = digitalRead(ROTB);
  enc_state = (enc_b << 1) | enc_a;
  interrupts();
}

// init receiver
void init_recv() {
  recv.begin();
}

// program setup
void setup() {
  init_timer0();
  init_pins();
  init_uart();
  init_encoder();
  init_i2c();
  init_oled();
  init_vfo();
  init_recv();
  if (ANY_PRESSED) {
    do_reset(FACTORY);
    while (SW1_PRESSED) wait_ms(DEBOUNCE);
    while (SW2_PRESSED) wait_ms(DEBOUNCE);
  } else {
    do_reset(SOFT);
  }
  show_version();
}

// main loop
void loop() {
  check_CAT();      // check CAT interface
  check_UI();       // check UI pushbutton
  check_menu();     // check for menu ops
}

