
/*
* ad7415.h - AD7415 library for Arduino
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

#ifndef AD7415_H_INCLUDED
#define AD7415_H_INCLUDED

#include "Arduino.h"
#include <stdint.h>
#include "SoftI2CMaster.h"


enum AD7415_Status
{
  AD7415_I2C_READ_OK,
  AD7415_I2C_WRITE_OK,
  AD7415_I2C_READ_ERROR,
  AD7415_I2C_WRITE_ERROR,
};


enum AD7415_Registers
{
    ad7415_reg_temp_value = 0x00,
    ad7415_reg_configuration = 0x01,
};
    
class AD7415
{
public:
  
  void init ();
  AD7415_Status read_temperature(float *temp);
  AD7415_Status writereg_u8 (AD7415_Registers reg, uint8_t data);
  AD7415_Status readreg_u16 (AD7415_Registers reg, uint16_t *data);

private:

  SoftI2CMaster SoftWire;
  int ad7415_i2c_addr = 0x4a;
  
};

#endif        // AD7415_H_INCLUDED

