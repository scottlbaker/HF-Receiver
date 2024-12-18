
// ============================================================================
//
// i2c0.h   - I2C library
//
// ============================================================================

#include <Arduino.h>
#include <inttypes.h>

#ifndef I2C0_H
#define I2C0_H

#define START           0x08
#define REPEATED_START  0x10
#define MT_SLA_ACK	0x18
#define MT_SLA_NACK	0x20
#define MT_DATA_ACK     0x28
#define MT_DATA_NACK    0x30
#define MR_SLA_ACK	0x40
#define MR_SLA_NACK	0x48
#define MR_DATA_ACK     0x50
#define MR_DATA_NACK    0x58
#define LOST_ARBTRTN    0x38
#define TW_STATUS       (TWSR0 & 0xF8)
#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)

class I2C0 {
  public:
    I2C0();
    void begin();
    void end();
    void write(uint8_t, uint8_t, uint8_t);
    void write(uint8_t, uint8_t, uint8_t*, uint8_t);
    void writezeros(uint8_t, uint8_t, uint8_t);
    void writeones(uint8_t, uint8_t, uint8_t);
    uint8_t read(uint8_t, uint8_t);

  private:
    uint8_t start();
    uint8_t sendAddress(uint8_t);
    uint8_t sendByte(uint8_t);
    uint8_t receiveByte();
    uint8_t stop();
    void lockUp();
};

extern I2C0 I2c0;

#endif

