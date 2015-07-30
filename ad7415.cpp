
/*
* ad7415.cpp - AD7415 library for Arduino
* Copyright (c) 2015 Otelo eGen - Georg Ottinger (georg.ottinger@oteloegen.at)
* All rights reserved.
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "ad7415.h"

void AD7415::init()
{
  
  writereg_u8(ad7415_reg_configuration, 0x40); //powerup, sda and scl filtering enabled
}

AD7415_Status AD7415::writereg_u8 (AD7415_Registers reg, uint8_t data)
{

  SoftWire.beginTransmission (ad7415_i2c_addr);
  SoftWire.write (reg);
  SoftWire.write (data);
  SoftWire.endTransmission ();

  return AD7415_I2C_WRITE_OK;
}

AD7415_Status AD7415::readreg_u16 (AD7415_Registers reg, uint16_t *data)
{

  SoftWire.beginTransmission (ad7415_i2c_addr);
  SoftWire.write (reg);
  SoftWire.endTransmission ();

  SoftWire.requestFrom (ad7415_i2c_addr);  
  
  *data = (uint16_t) SoftWire.read () << 8; // receive a byte as character
  *data |= SoftWire.readLast ();

  SoftWire.endTransmission();
  
  return AD7415_I2C_READ_OK;
}



AD7415_Status AD7415::read_temperature(float *temp)
{
  uint16_t value;
  
  if(readreg_u16(ad7415_reg_temp_value, &value) == AD7415_I2C_READ_ERROR)
    return AD7415_I2C_READ_ERROR;

  value >>= 6;

  *temp = (float)(value & 0x1FF) * 0.25;
  if( value & 0x200) 
    *temp -= 128.0;

}


