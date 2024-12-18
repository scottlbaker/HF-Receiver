
// ============================================================================
//
// i2c0.cpp   - I2C library
//
// ============================================================================

#include <Arduino.h>
#include <inttypes.h>
#include "globals.h"
#include "i2c0.h"

I2C0::I2C0() {
}

// Public Methods

void I2C0::begin() {
  pinMode(SDA0, INPUT);
  pinMode(SCL0, INPUT);
  TWBR0 = 0x10;   // bitrate = ((F_CPU / 400000) - 16) / 2
  TWSR0 = 0x00;   // prescaler = 1
  TWCR0 = 0x44;   // init TWI
}

void I2C0::end() {
  TWCR0 = 0;
}

void I2C0::write(uint8_t address, uint8_t registerAddress, uint8_t data) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  sendByte(data);
  stop();
}

void I2C0::write(uint8_t address, uint8_t registerAddress, uint8_t *data, uint8_t numberBytes) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  for (uint8_t i = 0; i < numberBytes; i++) sendByte(data[i]);
  stop();
}

void I2C0::writezeros(uint8_t address, uint8_t registerAddress, uint8_t numberBytes) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  for (uint8_t i = 0; i < numberBytes; i++) sendByte(0);
  stop();
}

void I2C0::writeones(uint8_t address, uint8_t registerAddress, uint8_t numberBytes) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  for (uint8_t i = 0; i < numberBytes; i++) sendByte(0xff);
  stop();
}

uint8_t I2C0::read(uint8_t address, uint8_t registerAddress) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  start();
  sendAddress(SLA_R(address));
  receiveByte();
  stop();
  return(TWDR);
}

// Private Methods

uint8_t I2C0::start() {
  TWCR0 = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while (!(TWCR0 & (1<<TWINT)));
  if ((TW_STATUS == START) || (TW_STATUS == REPEATED_START)) {
    return(0);
  }
  if (TW_STATUS == LOST_ARBTRTN) {
    uint8_t bufferedStatus = TW_STATUS;
    lockUp();
    return(bufferedStatus);
  }
  return(TW_STATUS);
}

uint8_t I2C0::sendAddress(uint8_t i2cAddress) {
  TWDR0 = i2cAddress;
  TWCR0 = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR0 & (1<<TWINT)));
  if ((TW_STATUS == MT_SLA_ACK) || (TW_STATUS == MR_SLA_ACK)) {
    return(0);
  }
  uint8_t bufferedStatus = TW_STATUS;
  if ((TW_STATUS == MT_SLA_NACK) || (TW_STATUS == MR_SLA_NACK)) {
    stop();
    return(bufferedStatus);
  } else {
    lockUp();
    return(bufferedStatus);
  }
}

uint8_t I2C0::sendByte(uint8_t i2cData) {
  TWDR0 = i2cData;
  TWCR0 = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR0 & (1<<TWINT)));
  if (TW_STATUS == MT_DATA_ACK) {
    return(0);
  }
  uint8_t bufferedStatus = TW_STATUS;
  if (TW_STATUS == MT_DATA_NACK) {
    stop();
    return(bufferedStatus);
  } else {
    lockUp();
    return(bufferedStatus);
  }
}

uint8_t I2C0::receiveByte() {
  TWCR0 = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR0 & (1<<TWINT)));
  if (TW_STATUS == LOST_ARBTRTN) {
    uint8_t bufferedStatus = TW_STATUS;
    lockUp();
    return(bufferedStatus);
  }
  return(TW_STATUS);
}

uint8_t I2C0::stop() {
  TWCR0 = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
  while ((TWCR0 & (1<<TWSTO)));
  return(0);
}

void I2C0::lockUp() {
  TWCR0 = 0x00;   // release SDA and SCL
  TWCR0 = 0x44;   // init TWI
}

I2C0 I2c0 = I2C0();

