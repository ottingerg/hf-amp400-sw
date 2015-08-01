/*
* hfamp400.h - HFamp400 library for Arduino 
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


#ifndef HFAMP400_H_INCLUDED
#define HFAMP400_H_INCLUDED

#include "Arduino.h"
#include <stdint.h>
#include "ad7294.h"
#include "ad7415.h"
#include "hfpowercal.h"

#define EE_CURRENT_OFFSET 0xF8

#define HFAMP400_DEFAULT_VOLTAGE_SCALING ((60.0 / (float)0xFFF) * 1.0326) //correction factor determined empirically
#define HFAMP400_DEFAULT_CURRENT_SCALING ((12.5 / (float)0xFFF))


#define ALERT_PIN 24
#define FUSE_CH_1_AC_CHARGE_PIN 48
#define FUSE_CH_1_CLOCK_PIN 46
#define FUSE_CH_2_AC_CHARGE_PIN 49
#define FUSE_CH_2_CLOCK_PIN 47

#define HFAMP400_FUSE_RESET_CHARGETIME 30
#define AD7294_ADC_RESULT_VALID_US 100000 // 100ms


#define ALERT_MASK_TEMP_D1 (ADV7294_ALERT_TSENSE_1_HIGH | ADV7294_ALERT_TSENSE_1_LOW)
#define ALERT_MASK_TEMP_D2 (ADV7294_ALERT_TSENSE_2_HIGH | ADV7294_ALERT_TSENSE_2_LOW)
#define ALERT_MASK_TEMP_INT (ADV7294_ALERT_TSENSE_INT_HIGH | ADV7294_ALERT_TSENSE_INT_LOW)
#define ALERT_MASK_HFPOWER_RETURN_1 (ADV7294_ALERT_VIN_1_HIGH | ADV7294_ALERT_VIN_1_LOW) 
#define ALERT_MASK_HFPOWER_RETURN_2 (ADV7294_ALERT_VIN_3_HIGH | ADV7294_ALERT_VIN_3_LOW)
#define ALERT_MASK_FUSE_1 (ADV7294_ALERT_VIN_0_HIGH | ADV7294_ALERT_VIN_0_LOW)
#define ALERT_MASK_FUSE_2 (ADV7294_ALERT_VIN_2_HIGH | ADV7294_ALERT_VIN_2_LOW)

 

enum HFAMP400_Status
{
  OK,
  ERROR_COMM,
  ERROR_RANGE,
  ERROR_CHANNEL,
};

enum ALERT_LIMIT
{
  ALERT_TEMP_D1,
  ALERT_TEMP_D2,
  ALERT_TEMP_AD7294,
  ALERT_HFPOWER_RETURN_1,
  ALERT_HFPOWER_RETURN_2,
  ALERT_CURRENT_FUSE_1,
  ALERT_CURRENT_FUSE_2,
};

enum GATEBIAS_CHANNEL
{
  GATEBIAS_CH_VOUT_A=1,
  GATEBIAS_CH_VOUT_B=2,
  GATEBIAS_CH_VOUT_C=3,
  GATEBIAS_CH_VOUT_D=4
  
};

enum TEMPERATURE_CHANNEL
{
  TEMP_CH_D1=1,
  TEMP_CH_D2=2,
  TEMP_CH_AD7294=3,
  TEMP_CH_BOARD=4,
};

enum FUSE_CHANNEL
{
  FUSE_CH_1=0,
  FUSE_CH_2=1,
  FUSE_CH_3=2
  
};

class HFAMP400
{
public:
  AD7294 ad7294;
  AD7415 ad7415;
  HFPOWERCAL hfpowercal;

  void init ();
  HFAMP400_Status get_temperature(TEMPERATURE_CHANNEL channel, float *temp);
  
  HFAMP400_Status get_fuse_current(FUSE_CHANNEL channel, float *current);
  HFAMP400_Status get_fuse_voltage(FUSE_CHANNEL channel, float *voltage);
  HFAMP400_Status get_hfpower(HFPOWER_CHANNEL channel, float *power);
  HFAMP400_Status get_ad7294_adcvalue(AD7294_ADC_CHANNEL channel, int *adcvalue);
  HFAMP400_Status get_adl5513_voltage(HFPOWER_CHANNEL channel, float *voltage);

  HFAMP400_Status set_gatebias(GATEBIAS_CHANNEL channel, float volt);
  HFAMP400_Status set_current_scaling(FUSE_CHANNEL channel, float factor);
  HFAMP400_Status set_voltage_scaling(FUSE_CHANNEL channel, float factor);
  HFAMP400_Status set_hfpower_conversion(HFPOWER_CHANNEL channel, HFPOWER_CONVERSION hf_conv);
  HFAMP400_Status set_hfpower_modulation_factor(HFPOWER_CHANNEL channel, float factor);
  HFAMP400_Status set_working_frequency(float frequency);
  HFAMP400_Status set_alert_limit(ALERT_LIMIT limit, float max);
  HFAMP400_Status set_alert_limit(ALERT_LIMIT limit, float min, float max);

  
  int read_line(char* buffer, int bufsize);
  void run_hfpower_calibration(void);
  void run_hfpower_calibration(uint8_t mask);
  void reset_gatebiasing(void);
  HFAMP400_Status reset_fuse(FUSE_CHANNEL channel);
  HFAMP400_Status run_current_offset_calibration(void);
  
private:
  float working_frequency = 100e6;

  uint16_t current_offset[3] = {
    0,
    0,
    0,
  };
  
  float current_factor[3] = {
    HFAMP400_DEFAULT_CURRENT_SCALING,  // FUSE 1 - INA207 and 1mOhm shunt outputs 2.5V (max of AD7294 ADC) at 50A
    HFAMP400_DEFAULT_CURRENT_SCALING,  // FUSE 2 - INA207 and 1mOhm shunt outputs 2.5V (max of AD7294 ADC) at 50A
    HFAMP400_DEFAULT_CURRENT_SCALING,  // FUSE 3 - INA207 and 1mOhm shunt outputs 2.5V (max of AD7294 ADC) at 50A
  };
  float voltage_factor[3] = {
    HFAMP400_DEFAULT_VOLTAGE_SCALING,
    HFAMP400_DEFAULT_VOLTAGE_SCALING,
    HFAMP400_DEFAULT_VOLTAGE_SCALING,
  };
  float hfpower_modulation_factor[4] = {
    1,
    1,
    1,
    1,
  };
  HFPOWER_CONVERSION hfpower_conversion[4] = {
    {0.02, 1.0},
    {0.02, 1.0},
    {0.02, 1.0},
    {0.02, 1.0},
  };
  
  
  
  bool checkboundaries( float var, float min, float max);
  bool checkboundaries( int var, int min, int max);
  float voltage_to_hfpower(HFPOWER_CHANNEL channel,float voltage);
  uint16_t hfpower_float_to_limitreg(HFPOWER_CHANNEL channel,float power);
  void reset_fuse_pins(int pin_ac, int pin_clock);
};

extern HFAMP400 hfamp400;

#endif				// HFAMP400_H_INCLUDED

