
// ============================================================================
//
// recv.cpp   - SSB receiver library
//
// ============================================================================

#include <Arduino.h>
#include <inttypes.h>
#include "recv.h"

#pragma GCC push_options
#pragma GCC optimize ("Ofast")  // compiler-optimization for speed

RECV::RECV() {
}

// Public Methods

static int16_t ocomb, ozi1, ozi2;

extern uint8_t radiomode;   // radio mode
extern uint8_t volume;      // audio volume
extern uint8_t filterbw;    // filter bandwidth
extern uint8_t dg_attn;     // digital attenuation
extern uint8_t agc;         // auto gain control
extern uint8_t rxstate;     // rx state

void RECV::begin() {
  init_adc();
  init_dac();
  set_dac_sample_rate(78125);
  set_adc_sample_rate(62500);  // start timer2 ADC sample clock
  set_dac_audio_enable(true);  // speaker output enable
}

void RECV::end() {
}

// Hilbert transform I
int16_t RECV::hilb_i(int16_t ac) {
  static int16_t v[7];  // delay-line to match Hilbert transform on Q branch
  for (int8_t i = 0; i != 6; i++) v[i] = v[i+1]; v[6] = ac;
  return v[0];
}

// Hilbert transform Q
int16_t RECV::hilb_q(int16_t ac) {
  static int16_t v[14];  // delay-line
  for (int8_t i = 0; i != 13; i++) v[i] = v[i+1]; v[13] = ac;
  return ((v[0] - v[13]) + (v[2] - v[12]) * 4) / 64 + ((v[4] - v[10]) + (v[6] - v[8])) / 8 + ((v[4] - v[10]) * 5 - (v[6] - v[8]) ) / 128 + (v[6] - v[8]) / 2;
}

// AGC
int16_t RECV::agc_fast(int16_t in) {
  static int16_t gain = 1024;
  int16_t agcout = (gain >= 1024) ? (gain >> 10) * in : in;
  int16_t accum = (1 - abs(agcout >> 10));
  if ((INT16_MAX - gain) > accum) gain = gain + accum;
  if (gain < 1) gain = 1;
  return agcout;
}

void RECV::dac_upsample(int16_t ac) {
  static int16_t ozd1, ozd2;  // output stage
  int16_t od1 = ac - ozd1;    // comb section
  ocomb = od1 - ozd2;
  ozd2 = od1;
  ozd1 = ac;
}

// sample processing
void RECV::process(int16_t i, int16_t q) {
  static int16_t ac3;
  dac_upsample(ac3);
  int16_t qh = hilb_q(q >> 2);
  int16_t ih = hilb_i(i >> 2);
  int16_t ac = (radiomode == USB) ? -(ih - qh) : -(ih + qh);
  ac = filter(ac);
  if (agc == FAST) ac = agc_fast(ac);
  ac = ac >> (16 - volume);
  ac3 = min(max(ac, -(1<<9)), (1<<9)-1 );
}

// init ADC
void RECV::init_adc() {
  DIDR0 |= 0xc0; // disable digital input for ADC6 and ADC7
  ADCSRA = 0x84; // ADEN=0x80 ADPS=0x04 (divide by 16)
  ADCSRB = 0;    // enable with prescaler
}

// init DAC
void RECV::init_dac() {
  TCCR1A = (1 << WGM11);
  TCCR1B = (1 << CS10) | (1 << WGM13) | (1 << WGM12); // Mode 14 - Fast PWM
}

// enable/disable DAC audio
void RECV::set_dac_audio_enable(bool val) {
  if (val) {
    TCCR1A |= (1 << COM1A1);
    pinMode(TONE, OUTPUT);
  } else {
    pinMode(TONE, INPUT);
    TCCR1A &= ~(1 << COM1A1);
  }
}

// PWM value range (fs>78431):  Fpwm = F_CPU / [Prescaler * (1 + TOP)]
void RECV::set_dac_sample_rate(uint32_t fs) {
  ICR1L = min(255, F_CPU / fs);
  ICR1H = 0x00;
}

// init ADC and set sample rate
void RECV::set_adc_sample_rate(uint16_t fs) {
  ASSR &= ~(1 << AS2);               // timer2 clocked by the I/O clock
  TCNT2 = 0;                         // clear the counter
  TCCR2A = (1 << WGM21);             // mode 2 - clear on compare match
  TCCR2B = (1 << CS22);              // 64 prescaler
  OCR2A = ((F_CPU / 64) / fs) - 1;   // OCRn = (F_CPU / pre-scaler / fs) - 1;
  TIMSK2 |= (1 << OCIE2A);           // enable TIMER2_COMPA interrupt
}

// returns unbiased ADC input
int16_t RECV::get_adc(uint8_t adcpin) {
  ADMUX = (adcpin - 14) | (1 << REFS1) | (1 << REFS0);
  ADCSRA |= (1 << ADSC);
  return ADC - 511;
}

// sample interpolation by averaging
int16_t RECV::sample_corr(int16_t ac) {
  static int16_t prev_adc;
  return (prev_adc + ac) / 2, prev_adc = ac;
}

void RECV::load_dac_audio() {
  ozi1 = ocomb + ozi1;
  ozi2 = ozi1 + ozi2;         // integrator section
  OCR1AL = min(max((ozi2 >> 5) + 128, 0), 255);
}

// sample processing state machine
void RECV::sample_dsp() {
  int16_t ac;
  static int16_t c[13];
  if (rxstate == 0) {
    ac = sample_corr(get_adc(QSDI));
    int16_t i_s1za0 = (ac + (c[0] + c[1]) * 3 + c[2]) >> 1;
    c[0] = ac;
    int16_t ac2 = (i_s1za0 + (c[3] + c[4]) * 3 + c[5]);
    c[3] = i_s1za0;
    process(ac2, c[12]);
  } else if (rxstate == 1) {
    ac = get_adc(QSDQ);
    load_dac_audio();
    c[8] = c[7];
    c[7] = ac;
  } else if (rxstate == 2) {
    ac = sample_corr(get_adc(QSDI));
    c[2] = c[1];
    c[1] = ac;
  } else if (rxstate == 3) {
    ac = get_adc(QSDQ);
    load_dac_audio();
    c[11] = c[10];
    c[10] = (ac + (c[6] + c[7]) * 3 + c[8]) >> 1;
    c[6] = ac;
  } else if (rxstate == 4) {
    ac = sample_corr(get_adc(QSDI));
    c[5] = c[4];
    c[4] = (ac + (c[0] + c[1]) * 3 + c[2]) >> 1;
    c[0] = ac;
  } else if (rxstate == 5) {
    ac = get_adc(QSDQ);
    load_dac_audio();
    c[8] = c[7];
    c[7] = ac;
  } else if (rxstate == 6) {
    ac = sample_corr(get_adc(QSDI));
    c[2] = c[1];
    c[1] = ac;
  } else if (rxstate == 7) {
    ac = get_adc(QSDQ);
    load_dac_audio();
    int16_t q_s1za0 = (ac + (c[6] + c[7]) * 3 + c[8]) >> 1;
    c[6] = ac;
    c[12] = (q_s1za0 + (c[9] + c[10]) * 3 + c[11]);
    c[9] = q_s1za0;
  } else {
    rxstate=0;
  }
  rxstate++;
  if (rxstate > 7) rxstate=0;
}

#define NTAPS 11

// bandwidth filter FIR coefficients
// calculated by WinFilter
int16_t RECV::filter(int16_t ac) {
  uint16_t c[NTAPS];
  uint8_t gain;
  static int32_t x[NTAPS];
  int32_t t;
  int32_t y=0;
  int16_t y2=0;

  // shift samples
  for(uint8_t n=NTAPS-1; n>0; n--) x[n] = x[n-1];
  x[0] = ac;

  switch (filterbw) {
    case BW1500:
      t = x[0]+x[10];                     // 0x006
      y += (t<<2)+(t<<1);
      t = x[1]+x[9];                      // 0x026
      y -= (t<<5)+(t<<2)+(t<<1);
      t = x[2]+x[8];                      // 0x074
      y -= (t<<6)+(t<<5)+(t<<4)+(t<<2);
      t = x[3]+x[7];                      // 0x01a
      y += (t<<4)+(t<<3)+(t<<1);
      t = x[4]+x[6];                      // 0x470
      y += (t<<10)+(t<<6)+(t<<5)+(t<<4);
      t = x[5];                           // 0x810
      y += (t<<11)+(t<<4);
      break;
    case BW2000:
      t = x[0]+x[10];                     // 0x003
      y += (t<<1)+t;
      t = x[1]+x[9];                      // 0x028
      y += (t<<5)+(t<<3);
      t = x[2]+x[8];                      // 0x03a
      y -= (t<<5)+(t<<4)+(t<<3)+(t<<1);
      t = x[3]+x[7];                      // 0x114
      y -= (t<<8)+(t<<4)+(t<<2);
      t = x[4]+x[6];                      // 0x430
      y += (t<<10)+(t<<5)+(t<<4);
      t = x[5];                           // 0x9e0
      y += (t<<11)+(t<<8)+(t<<7)+(t<<6)+(t<<5);
      break;
    case BW2500:
      t = x[0]+x[10];                     // 0x018
      y -= (t<<4)+(t<<3);
      t = x[1]+x[9];                      // 0x004
      y -= (t<<2);
      t = x[2]+x[8];                      // 0x090
      y += (t<<7)+(t<<4);
      t = x[3]+x[7];                      // 0x1e0
      y -= (t<<8)+(t<<7)+(t<<6)+(t<<5);
      t = x[4]+x[6];                      // 0x390
      y += (t<<9)+(t<<8)+(t<<7)+(t<<4);
      t = x[5];                           // 0xbc0
      y += (t<<11)+(t<<9)+(t<<8)+(t<<7)+(t<<6);
      break;
    case BWFULL:
      t = x[0]+x[10];                     // 0x0a8
      y += (t<<7)+(t<<5)+(t<<3);
      t = x[1]+x[9];                      // 0x0b8
      y -= (t<<7)+(t<<5)+(t<<4)+(t<<3);
      t = x[2]+x[8];                      // 0x0c6
      y += (t<<7)+(t<<6)+(t<<2)+(t<<1);
      t = x[3]+x[7];                      // 0x0d2
      y -= (t<<7)+(t<<6)+(t<<4)+(t<<1);
      t = x[4]+x[6];                      // 0x0da
      y += (t<<7)+(t<<6)+(t<<4)+(t<<3)+(t<<1);
      t = x[5];                           // 0xe80
      y += (t<<11)+(t<<10)+(t<<9)+(t<<7);
      break;
    default:
      return(ac);
      break;
  }
  gain = 11 - dg_attn;
  y2 = y>>gain;
  return(y2);
}

#pragma GCC pop_options

