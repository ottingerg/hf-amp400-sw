/*
 * SoftI2CMaster.h -- Multi-instance software I2C Master library
 * 
 * 2010-2012 Tod E. Kurt, http://todbot.com/blog/
 * 2014, by Testato: update library and examples for follow Wireâ€™s API of Arduino IDE 1.x
 * 2015, by Georg Ottinger: replace Pin Operations with generic Arduino digitalWrite, digitalRead, pinMode - to allow usage with Arduino Due
 *
 */

#ifndef SoftI2CMaster_h
#define SoftI2CMaster_h

#include <inttypes.h>

#define _SOFTI2CMASTER_VERSION 13  // software version of this library


class SoftI2CMaster
{

private:
  // per object data
  uint8_t _sclPin=26;
  uint8_t _sdaPin=25;

 
  // private methods

  void i2c_writebit( uint8_t c );
  uint8_t i2c_readbit(void);
  void i2c_init(void);
  void i2c_start(void);
  void i2c_repstart(void);
  void i2c_stop(void);
  uint8_t i2c_write( uint8_t c );
  uint8_t i2c_read( uint8_t ack );
  
public:
  // public methods
  SoftI2CMaster();
  SoftI2CMaster(uint8_t sclPin, uint8_t sdaPin);

  void setPins(uint8_t sclPin, uint8_t sdaPin);

  uint8_t beginTransmission(uint8_t address);
  uint8_t beginTransmission(int address);
  uint8_t endTransmission(void);
  uint8_t write(uint8_t);
  void write(uint8_t*, uint8_t);
  void write(int);
  void write(char*);

  uint8_t requestFrom(int address);
  uint8_t requestFrom(uint8_t address);
  uint8_t read( uint8_t ack );
  uint8_t read();
  uint8_t readLast();

};



#endif

