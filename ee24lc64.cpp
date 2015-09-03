/* 
  *  Use the I2C bus with EEPROM 24LC64 
  *  Sketch:    eeprom.pde
  *  
  *  Author: hkhijhe
  *  Date: 01/10/2010
  * 
  *   
  */

#include <Wire.h>
  #include "ee24lc64.h" //I2C library


byte EE24LC64::init(void)
{
  byte error;
  Wire.begin();
  Wire.beginTransmission(eeprom_addr);
  error = Wire.endTransmission();
  return error;
}

byte EE24LC64::init(int addr)
{
  eeprom_addr = addr;
  return init();
}


byte EE24LC64::init(int addr,bool set_16bit_addr)
{
  use_16bit_addr = set_16bit_addr;
  return init(addr);
}


void EE24LC64::write_buffer(unsigned int eeaddress, byte *data, byte length ) {
  write_buffer(eeprom_addr, eeaddress,data,length);
}

void EE24LC64::write_buffer(int deviceaddress, unsigned int eeaddress, byte *data, byte length ) {
  int i;
  for (i = 0; i  < length; i++)
  {
    write_byte(deviceaddress, eeaddress+i, data[i]);
  }
}

void EE24LC64::write_byte(unsigned int eeaddress, byte data ) {
  write_byte(eeprom_addr, eeaddress,data);
}

  void EE24LC64::write_byte( int deviceaddress, unsigned int eeaddress, byte data ) {
    int rdata = data;
    
    Wire.beginTransmission(deviceaddress);
    if(use_16bit_addr) Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.write(rdata);
    Wire.endTransmission();
    
    delay(5);
  }

  // WARNING: address is a page address, 6-bit end will wrap around
  // also, data can be maximum of about 30 bytes, because the Wire library has a buffer of 32 bytes

  void EE24LC64::write_page(unsigned int eeaddresspage, byte* data, byte length ) {
    write_page(eeprom_addr,eeaddresspage, data, length );
  }
  
  void EE24LC64::write_page( int deviceaddress, unsigned int eeaddresspage, byte* data, byte length ) {
    
    Wire.beginTransmission(deviceaddress);
    if(use_16bit_addr) Wire.write((int)(eeaddresspage >> 8)); // MSB
    Wire.write((int)(eeaddresspage & 0xFF)); // LSB
    byte c;
    for ( c = 0; c < length; c++)
      Wire.write(data[c]);
    Wire.endTransmission();
    
  }

byte EE24LC64::read_byte( unsigned int eeaddress ) 
{
  return read_byte( eeprom_addr, eeaddress );
}

  byte EE24LC64::read_byte( int deviceaddress, unsigned int eeaddress ) {
    byte rdata = 0xFF;
    
    Wire.beginTransmission(deviceaddress);
    if(use_16bit_addr) Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
    
    delay(5);

    
    Wire.requestFrom(deviceaddress,1);
    if (Wire.available()) rdata = Wire.read();
    
    return rdata;
  }


 void EE24LC64::read_buffer( unsigned int eeaddress, byte *buffer, int length ) {
  read_buffer(eeprom_addr, eeaddress, buffer, length );
 }
 
  // maybe let's not read more than 30 or 32 bytes at a time!
  void EE24LC64::read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length ) {
    
    Wire.beginTransmission(deviceaddress);
    if(use_16bit_addr) Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(deviceaddress,length);
    int c = 0;
    for ( c = 0; c < length; c++ )
      if (Wire.available()) buffer[c] = Wire.read();
    
  }



