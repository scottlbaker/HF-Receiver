
// ============================================================================
// globals.h :: gobal variables
// ============================================================================

#ifndef GLOBAL_H
#define GLOBAL_H

// Arduino Pins
#define QSDQ     21   // PE3/ADC7  Q         (pin 22)
#define QSDI     20   // PE2/ADC6  I         (pin 19)
#define XLED      5   // PD5   diag LED      (pin  9)
#define SW1       3   // PD3   push button   (pin  1)
#define SW2       4   // PD4   push button   (pin  2)
#define SW3       6   // PD6   push button   (pin 10)
#define ROTA      7   // PD7   encoder A     (pin 11)
#define ROTB      8   // PB0   encoder B     (pin 12)
#define TONE      9   // PB1   sidetone      (pin 13)
#define RXD       0   // PD0   serial rx     (pin 30)
#define TXD       1   // PD1   serial tx     (pin 31)

#define SDA0     18   // PC4   i2c bus #0    (pin 27)
#define SCL0     19   // PC5   i2c bus #0    (pin 28)
#define SDA1     23   // PE0   i2c bus #1    (pin  3)
#define SCL1     24   // PE1   i2c bus #1    (pin  6)

// generic
#define OFF      0
#define ON       1
#define NO       0
#define YES      1
#define FALSE    0
#define TRUE     1

// reset types
#define FACTORY  0    // factory reset
#define SOFT     1    // soft reset

// radio modes
#define USB  0
#define LSB  1
#define CW   2

// bandwidth filter settings
#define BW1500  0
#define BW2000  1
#define BW2500  2
#define BWFULL  3

// agc modes
#define FAST  1
#define SLOW  2


#endif

