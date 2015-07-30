/*
* ad7294.h - AD7294 library for Arduino 
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

#ifndef AD7294_H_INCLUDED
#define AD7294_H_INCLUDED

#include "Arduino.h"
#include <stdint.h>
#include <Wire.h>


#define AD7294_ADC_RESULT_VALID_US 100000 // 100ms

enum AD7294_ADC_CHANNEL
{
  AD7294_ADC_CH_VIN_0=0,
  AD7294_ADC_CH_VIN_1=1,
  AD7294_ADC_CH_VIN_2=2,
  AD7294_ADC_CH_VIN_3=3,
  AD7294_ADC_CH_ISENSE_1=4,
  AD7294_ADC_CH_ISENSE_2=5,
  AD7294_ADC_CH_TSENSE_1=6,
  AD7294_ADC_CH_TSENSE_2=7
};

#define ADV7294_ALERT_VIN_3_HIGH 0x800000
#define ADV7294_ALERT_VIN_3_LOW 0x400000
#define ADV7294_ALERT_VIN_2_HIGH 0x200000
#define ADV7294_ALERT_VIN_2_LOW 0x100000
#define ADV7294_ALERT_VIN_1_HIGH 0x080000
#define ADV7294_ALERT_VIN_1_LOW 0x040000
#define ADV7294_ALERT_VIN_0_HIGH 0x020000
#define ADV7294_ALERT_VIN_0_LOW 0x010000
#define ADV7294_ALERT_ISENSE_2_OVERRANGE 0x002000
#define ADV7294_ALERT_ISENSE_1_OVERRANGE 0x002000
#define ADV7294_ALERT_ISENSE_2_HIGH 0x000800
#define ADV7294_ALERT_ISENSE_2_LOW 0x000400
#define ADV7294_ALERT_ISENSE_2_HIGH 0x000200
#define ADV7294_ALERT_ISENSE_2_LOW 0x000100
#define ADV7294_ALERT_OPENDIODE 0x000080
#define ADV7294_ALERT_OVERTEMP 0x000040
#define ADV7294_ALERT_TSENSE_INT_HIGH 0x000020
#define ADV7294_ALERT_TSENSE_INT_LOW 0x000010
#define ADV7294_ALERT_TSENSE_2_HIGH 0x000008
#define ADV7294_ALERT_TSENSE_2_LOW 0x000004
#define ADV7294_ALERT_TSENSE_1_HIGH 0x000002
#define ADV7294_ALERT_TSENSE_1_LOW 0x000001




struct
{
    uint16_t value;
    uint32_t timestamp;
} typedef ADC_RESULTS;


enum AD7294_Limits
{
  AD7294_LIMIT_VIN_0=0,
  AD7294_LIMIT_VIN_1=1,
  AD7294_LIMIT_VIN_2=2,
  AD7294_LIMIT_VIN_3=3,
  AD7294_LIMIT_ISENSE_1=4,
  AD7294_LIMIT_ISENSE_2=5,
  AD7294_LIMIT_TSENSE_1=6,
  AD7294_LIMIT_TSENSE_2=7, 
  AD7294_LIMIT_TSENSE_INT=8
  
};

enum AD7294_Status
{
  AD7294_I2C_READ_OK,
  AD7294_I2C_WRITE_OK,
  AD7294_I2C_READ_ERROR,
  AD7294_I2C_WRITE_ERROR,
  AD7294_TIMEOUT
};

enum AD7294_Registers
{
    command = 0x00,
    adc_result = 0x01,
    dac_a_value = 0x01,
    tsense_1_result = 0x02,
    dac_b_value = 0x02,
    tsense_2_result = 0x03,
    dac_c_value = 0x03,
    tsense_int_result = 0x04,
    dac_d_value = 0x04,
    alert_status_a = 0x05,
    alert_status_b = 0x06,
    alert_status_c = 0x07,
    channel_sequence = 0x08,
    configuration = 0x09,
    powerdown = 0x0A,
    vin_0_datalow = 0x0b,
    vin_0_datahigh = 0x0c,
    vin_0_hysteresis = 0x0d,
    vin_1_datalow = 0x0e,
    vin_1_datahigh = 0x0f,
    vin_1_hysteresis = 0x10,
    vin_2_datalow = 0x11,
    vin_2_datahigh = 0x12,
    vin_2_hysteresis = 0x13,
    vin_3_datalow = 0x14,
    vin_3_datahigh = 0x15,
    vin_3_hysteresis = 0x16,
    isense_1_datalow = 0x17,
    isense_1_datahigh = 0x18,
    isense_1_hysteresis = 0x19,
    isense_2_datalow = 0x1A,
    isense_2_datahigh = 0x1B,
    isense_2_hysteresis = 0x1C,
    tsense_1_datalow = 0x1D,
    tsense_1_datahigh = 0x1E,
    tsense_1_hysteresis = 0x1F,
    tsense_2_datalow = 0x20,
    tsense_2_datahigh = 0x21,
    tsense_2_hysteresis = 0x22,
    tsense_int_datalow = 0x23,
    tsense_int_datahigh = 0x24,
    tsense_int_hysteresis = 0x25,
    tsense_1_offset = 0x26,
    tsense_2_offset = 0x27,
};



class AD7294
{
public:
  ADC_RESULTS adc_results[8];

  void init ();
  void init (uint8_t i2c_addr);
  AD7294_Status writereg_u16 (AD7294_Registers reg, uint16_t data);
  AD7294_Status readreg_u16 (AD7294_Registers reg, uint16_t *data);
  AD7294_Status writereg_u8 (AD7294_Registers reg, uint8_t data);
  AD7294_Status readreg_u8 (AD7294_Registers reg, uint8_t *data);
  AD7294_Status readagain_u16 (uint16_t *data);
  AD7294_Status readagain_u8 (uint8_t *data);

  AD7294_Status set_dac(int channel, uint16_t value);
  AD7294_Status read_temperature(int channel, float *temp);

  AD7294_Status readadcs_autocycling();
  AD7294_Status readadcs_autocycling(uint8_t mask);

  AD7294_Status set_limit( AD7294_Limits limit, uint16_t min_value, uint16_t max_value, uint16_t hyst);
  
  uint32_t get_alerts();
  void clear_alerts();
  uint16_t temp_float_to_limitreg(float temp);
  
private:
  int ad7294_i2c_addr = 0x61;
  
  uint8_t lsb (uint16_t w);
  uint8_t msb (uint16_t w);


};


#endif				// AD7294_H_INCLUDED
