
// ============================================================================
//
// i2c1.cpp   - I2C library
//
// ============================================================================

#include <Arduino.h>
#include <inttypes.h>
#include "globals.h"
#include "i2c1.h"

I2C1::I2C1() {
}

// Public Methods

void I2C1::begin() {
  pinMode(SDA1, INPUT);
  pinMode(SCL1, INPUT);
  TWBR1 = 0x10;   // bitrate = ((F_CPU / 400000) - 16) / 2
  TWSR1 = 0x00;   // prescaler = 1
  TWCR1 = 0x44;   // init TWI
}

void I2C1::end() {
  TWCR1 = 0;
}

void I2C1::write(uint8_t address, uint8_t registerAddress, uint8_t data) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  sendByte(data);
  stop();
}

void I2C1::write(uint8_t address, uint8_t registerAddress, uint8_t *data, uint8_t numberBytes) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  for (uint8_t i = 0; i < numberBytes; i++) sendByte(data[i]);
  stop();
}

void I2C1::writezeros(uint8_t address, uint8_t registerAddress, uint8_t numberBytes) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  for (uint8_t i = 0; i < numberBytes; i++) sendByte(0);
  stop();
}

void I2C1::writeones(uint8_t address, uint8_t registerAddress, uint8_t numberBytes) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  for (uint8_t i = 0; i < numberBytes; i++) sendByte(0xff);
  stop();
}

uint8_t I2C1::read(uint8_t address, uint8_t registerAddress) {
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

uint8_t I2C1::start() {
  TWCR1 = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while (!(TWCR1 & (1<<TWINT)));
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

uint8_t I2C1::sendAddress(uint8_t i2cAddress) {
  TWDR1 = i2cAddress;
  TWCR1 = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR1 & (1<<TWINT)));
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

uint8_t I2C1::sendByte(uint8_t i2cData) {
  TWDR1 = i2cData;
  TWCR1 = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR1 & (1<<TWINT)));
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

uint8_t I2C1::receiveByte() {
  TWCR1 = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR1 & (1<<TWINT)));
  if (TW_STATUS == LOST_ARBTRTN) {
    uint8_t bufferedStatus = TW_STATUS;
    lockUp();
    return(bufferedStatus);
  }
  return(TW_STATUS);
}

uint8_t I2C1::stop() {
  TWCR1 = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
  while ((TWCR1 & (1<<TWSTO)));
  return(0);
}

void I2C1::lockUp() {
  TWCR1 = 0x00;   // release SDA and SCL
  TWCR1 = 0x44;   // init TWI
}

I2C1 I2c1 = I2C1();

