
/*
* ad7294.cpp - AD7294 library for Arduino
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


#include "ad7294.h"

//#define DEBUG
//#define DEBUG_ALERT

void AD7294::init ()
{
    writereg_u8(powerdown, 0x06); // Power everything up except ISENSE
    writereg_u8(alert_status_a, 0xff); // clear alert status A reg
    writereg_u8(alert_status_b, 0xff); // clear alert status B reg
    writereg_u8(alert_status_c, 0xff); // clear alert status C reg
    writereg_u16(dac_a_value, 0x0000); // Set DAC A to 0V
    writereg_u16(dac_b_value, 0x0000); // Set DAC B to 0V
    writereg_u16(dac_c_value, 0x0000); // Set DAC C to 0V
    writereg_u16(dac_d_value, 0x0000); // Set DAC D to 0V
    writereg_u8(tsense_1_offset,0); //Use 2n3906 PNP transistor for Temp measurement
    writereg_u8(tsense_2_offset,0); //Use 2n3906 PNP transistor for Temp measurement
    writereg_u8(channel_sequence, 0x0f); // auto cycle all channels except ISENSE1, ISENSE2
    writereg_u16(configuration, 0x1004); // auto cycle mode, Vin Range = VRef (=2.5V), Alert pin active low
    set_limit(AD7294_LIMIT_VIN_0,0,0xFFF,0xFFE);
    set_limit(AD7294_LIMIT_VIN_1,0,0xFFF,0xFFE);
    set_limit(AD7294_LIMIT_VIN_2,0,0xFFF,0xFFE);
    set_limit(AD7294_LIMIT_VIN_3,0,0xFFF,0xFFE);
    set_limit(AD7294_LIMIT_ISENSE_1,0x800,0x7FF,0x7FE);
    set_limit(AD7294_LIMIT_ISENSE_2,0x800,0x7FF,0x7FE);
    set_limit(AD7294_LIMIT_TSENSE_1,0x400,0x3FF,0x3FE);
    set_limit(AD7294_LIMIT_TSENSE_2,0x400,0x3FF,0x3FE); 
    set_limit(AD7294_LIMIT_TSENSE_INT,0x400,0x3FF,0x3FE);
    
}

void AD7294::init (uint8_t i2c_addr)
{
  ad7294_i2c_addr = i2c_addr;
  init ();
}



AD7294_Status AD7294::readadcs_autocycling()
{
    uint8_t mask =  bit(AD7294_ADC_CH_VIN_0) |
                    bit(AD7294_ADC_CH_VIN_1) |
                    bit(AD7294_ADC_CH_VIN_2) |
                    bit(AD7294_ADC_CH_VIN_3);
                    
                    //bit(AD7294_ADC_CH_TSENSE_1) |
                    //bit(AD7294_ADC_CH_TSENSE_2);


    readadcs_autocycling(mask);
}


uint32_t AD7294::get_alerts()
{
    uint32_t alerts = 0;
    uint8_t a=0xFF;

    
    readreg_u8(alert_status_a,&a);
#ifdef DEBUG_ALERT
    Serial.print("alert a= ");
    Serial.println(a,HEX);
#endif

    alerts |= (uint32_t)a << 16;
    
    a=0xFF;
    readreg_u8(alert_status_b,&a);
 
 #ifdef DEBUG_ALERT
    Serial.print("alert b= ");
    Serial.println(a,HEX);
#endif
 
    alerts |= (uint32_t)a << 8;
    a=0xFF;
   
    readreg_u8(alert_status_c,&a);
 #ifdef DEBUG_ALERT
    Serial.print("alert c= ");
    Serial.println(a,HEX);
#endif
    alerts |= (uint32_t)a;

    return alerts;

}

void AD7294::clear_alerts()
{
    writereg_u8(alert_status_a, 0xff);
    writereg_u8(alert_status_b, 0xff);
    writereg_u8(alert_status_c, 0xff);
}


AD7294_Status AD7294::readadcs_autocycling(uint8_t mask)
{
  AD7294_Status res;
  uint8_t channel;
  uint16_t adc;
  uint16_t cnt = 1,timeout=100;

  res = readreg_u16(adc_result,&adc);
  if (res != AD7294_I2C_READ_OK)
    return res;


  do {
    channel = (msb(adc) >> 4) & 0x07;
    adc_results[channel].value = adc & 0x0fff;
    adc_results[channel].timestamp = micros();
    mask &= ~bit(channel);
    cnt++;
    res=readagain_u16(&adc);
    if (res != AD7294_I2C_READ_OK)
        return res;
    delayMicroseconds(65); //Hack:determined by experimentation to keep the needed readcycles low
    timeout--;
  } while (mask && timeout);

#ifdef DEBUG
    Serial.print("Read cycle needed: ");
    Serial.println(cnt);
#endif // DEBUG
   if(!timeout) return AD7294_TIMEOUT;
   return AD7294_I2C_READ_OK;
}

AD7294_Status AD7294::readagain_u16(uint16_t *data)
{
  

  Wire.requestFrom (ad7294_i2c_addr, 2);	// request 6 bytes from slave device #2

  if (Wire.available () != 2)
    return AD7294_I2C_READ_ERROR;

  while (Wire.available ())	// slave may send less than requested
    {
      *data = (uint16_t) Wire.read () << 8;	// receive a byte as character
      *data |= Wire.read ();
    }

  
  return AD7294_I2C_READ_OK;
}

AD7294_Status AD7294::readagain_u8(uint8_t *data)
{

  
  
  Wire.requestFrom (ad7294_i2c_addr, 1);	// request 6 bytes from slave device #2

  if (Wire.available () != 1)
    return AD7294_I2C_READ_ERROR;

  while (Wire.available ())	// slave may send less than requested
    {
      *data |= Wire.read ();
    }

  
  return AD7294_I2C_READ_OK;
}

AD7294_Status AD7294::readreg_u16 (AD7294_Registers reg, uint16_t *data)
{

  

  Wire.beginTransmission (ad7294_i2c_addr);
  Wire.write (reg);
  Wire.endTransmission (true);
  //delayMicroseconds(100);
    
  Wire.requestFrom (ad7294_i2c_addr, 2);	// request 6 bytes from slave device #2

  //delayMicroseconds(400);
  
  if (Wire.available () != 2)
    return AD7294_I2C_READ_ERROR;

  while (Wire.available ())	// slave may send less than requested
    {
      *data = (uint16_t) Wire.read () << 8;	// receive a byte as character
      *data |= Wire.read ();
    }
  
  
  
  return AD7294_I2C_READ_OK;
}

AD7294_Status AD7294::readreg_u8 (AD7294_Registers reg, uint8_t *data)
{

  

  //delayMicroseconds(500000);
  Wire.beginTransmission (ad7294_i2c_addr);
  Wire.write (reg);
  Wire.endTransmission ();
  //delayMicroseconds(500000);
  Wire.requestFrom (ad7294_i2c_addr, 1);	// request 6 bytes from slave device #2

  //delayMicroseconds(200);
  
  if (Wire.available () != 1)
  {

#ifdef DEBUG_ALERT 
    Serial.println("test");
#endif
    return AD7294_I2C_READ_ERROR;
  }
    

  while (Wire.available ())	// slave may send less than requested
    {
      *data = Wire.read ();
    }


#ifdef DEBUG_ALERT 
    Serial.print("readreg_u8 read ");
    Serial.println(*data,HEX);
#endif
  
  
  return AD7294_I2C_READ_OK;
}

AD7294_Status AD7294::writereg_u8 (AD7294_Registers reg, uint8_t data)
{
  
  
  Wire.beginTransmission (ad7294_i2c_addr);
  Wire.write (reg);
  Wire.write (data);
  Wire.endTransmission (true);
  
  

  return AD7294_I2C_WRITE_OK;
}

AD7294_Status AD7294::writereg_u16 (AD7294_Registers reg, uint16_t data)
{

  

  Wire.beginTransmission (ad7294_i2c_addr);
  Wire.write (reg);
  Wire.write (msb (data));
  Wire.write (lsb (data));
  Wire.endTransmission (true);

  
  
  return AD7294_I2C_WRITE_OK;
}

AD7294_Status AD7294::set_dac(int channel, uint16_t value)
{
   AD7294_Registers reg = dac_a_value; 

  switch(channel)
  {
    case 1:
      reg = dac_a_value;
      break;
    case 2:
      reg = dac_b_value;
      break;
    case 3:
      reg = dac_c_value;
      break;
    case 4:
      reg = dac_d_value;
      break;
  }

  return writereg_u16(reg, value);
}


AD7294_Status AD7294::read_temperature(int channel, float *temp)
{
  uint16_t value;
  AD7294_Registers reg = tsense_1_result; 

  switch(channel)
  {
    case 1:
      reg = tsense_1_result;
      break;
    case 2:
      reg = tsense_2_result;
      break;
    case 3:
      reg = tsense_int_result;
      break;  
  }
  
  if(readreg_u16(reg, &value) == AD7294_I2C_READ_ERROR)
    return AD7294_I2C_READ_ERROR;

  *temp = (float)lsb(value) * 0.25;
  if( value & 0x100) *temp += 128.0;
  if( value & 0x200) *temp -= 256.0;

  if( value & 0x400) Serial.println("open");

  return AD7294_I2C_READ_OK;
  
}

AD7294_Status AD7294::set_limit( AD7294_Limits limit, uint16_t min_value, uint16_t max_value, uint16_t hyst)
{
  int reg;
  AD7294_Status res;

  reg = (int)limit*3;
  reg += (int)vin_0_datalow;

  res = writereg_u16((AD7294_Registers)reg, min_value);
  if (res != AD7294_I2C_WRITE_OK) 
    return res;

  reg ++;
  
  res = writereg_u16((AD7294_Registers)reg, max_value);
  if (res != AD7294_I2C_WRITE_OK) 
    return res;
  
  reg ++;

  res = writereg_u16((AD7294_Registers)reg, hyst);

  return res;
  
}

uint16_t AD7294::temp_float_to_limitreg(float temp)
{
  uint16_t res=0;
  float t;

  t = temp;
  if(temp < 0)
  {
    t += 256.0;
    res |= 0x400;
  } 

  t *= 4;

  res |= ((uint16_t)t) & 0x3FF;

  return res;
}

uint8_t AD7294::msb(uint16_t w)
{
    return (w&0xff00)>>8;
}

uint8_t AD7294::lsb(uint16_t w)
{
    return (w&0xff);
}




