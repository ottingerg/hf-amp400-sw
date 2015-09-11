/*
* hfamp400.cpp - HFamp400 library for Arduino 
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

#include <Wire.h>
#include "hfamp400.h"

//#define DEBUG
//#define DEBUG_CURRENT

void HFAMP400::init ()
{
  analogReadResolution(12); //Set ADC Resulotion of Arduino Due to 12 Bit
  Wire.begin();
  ad7294.init();
  ad7415.init();
  hfpowercal.init();
  //Read current offset from EEPROM
  hfpowercal.eeprom.read_buffer(EE_VOLTAGE_CALIBRATION,(byte*)&voltage_factor, sizeof(voltage_factor));
  hfpowercal.eeprom.read_buffer(EE_CURRENT_CALIBRATION,(byte*)&current_factor, sizeof(current_factor));
 
  analogRead(A0); //Dummy ADC reads to avoid uninitialized ADCs
  analogRead(A1);
  analogRead(A2);
  analogRead(A3);
  analogRead(A4);
  analogRead(A5);
  set_working_frequency(100e6);
  pinMode(FUSE_CH_1_AC_CHARGE_PIN, OUTPUT); pinMode(FUSE_CH_1_CLOCK_PIN,OUTPUT);  //Fuse Reset Pins
  pinMode(FUSE_CH_2_AC_CHARGE_PIN, OUTPUT); pinMode(FUSE_CH_2_CLOCK_PIN,OUTPUT);  //Fuse Reset Pins

}


HFAMP400_Status HFAMP400::reset_fuse(FUSE_CHANNEL channel)
{
  if (!checkboundaries((int)channel, (int)(FUSE_CHANNEL)FUSE_CH_1, (int)(FUSE_CHANNEL)FUSE_CH_3))
    return ERROR_CHANNEL;  
    
  switch(channel)
  {
    case FUSE_CH_1:
      reset_fuse_pins(FUSE_CH_1_AC_CHARGE_PIN,FUSE_CH_1_CLOCK_PIN); 
      break;
      
    case FUSE_CH_2:
    case FUSE_CH_3:
      reset_fuse_pins(FUSE_CH_2_AC_CHARGE_PIN,FUSE_CH_2_CLOCK_PIN); 
      break;    
  }

  return OK;
}

void HFAMP400::reset_fuse_pins(int pin_ac, int pin_clock)
{
  int i;
  
  
  digitalWrite(pin_clock,LOW);

  for( i = 0; i < HFAMP400_FUSE_RESET_CHARGETIME; i++)
  {
    digitalWrite(pin_ac, HIGH);
    delayMicroseconds(500);  
    digitalWrite(pin_ac, LOW);
    delayMicroseconds(500);
  }

  digitalWrite(pin_clock,HIGH);
  delayMicroseconds(100);
  digitalWrite(pin_clock,LOW);
}

void HFAMP400::reset_gatebiasing(void)
{
  ad7294.writereg_u16(dac_a_value, 0x0);
  ad7294.writereg_u16(dac_b_value, 0x0);
  ad7294.writereg_u16(dac_c_value, 0x0);
  ad7294.writereg_u16(dac_d_value, 0x0);
}

HFAMP400_Status HFAMP400::set_gatebias(GATEBIAS_CHANNEL channel, float volt)
{
  uint16_t value;

  if (!checkboundaries((int)channel, (int)(GATEBIAS_CHANNEL)GATEBIAS_CH_VOUT_A, (int)(GATEBIAS_CHANNEL)GATEBIAS_CH_VOUT_D))
    return ERROR_CHANNEL;
    
  if (!checkboundaries(volt, (float)0.0, (float)5.0))
    return ERROR_RANGE;
    
  value = volt * (float)0xFFF / 5;

  if ( ad7294.set_dac(channel,value) == AD7294_I2C_WRITE_OK )
    return OK;
  else
    return ERROR_COMM;
}

HFAMP400_Status HFAMP400::get_ad7294_adcvalue(AD7294_ADC_CHANNEL channel, int *adcvalue)
{
  uint32_t now;
  
  if (!checkboundaries((int)channel, (int)(AD7294_ADC_CHANNEL)AD7294_ADC_CH_VIN_0, (int)(AD7294_ADC_CHANNEL)AD7294_ADC_CH_VIN_3))
    return ERROR_CHANNEL;  

  now = micros();

  if( (now - ad7294.adc_results[channel].timestamp) > AD7294_ADC_RESULT_VALID_US)
    if( ad7294.readadcs_autocycling() == AD7294_TIMEOUT)
      return ERROR_COMM;

  *adcvalue = ad7294.adc_results[channel].value;

  return OK;
  
}


HFAMP400_Status HFAMP400::get_temperature(TEMPERATURE_CHANNEL channel, float *temp)
{
  
  switch(channel)
  {
    case TEMP_CH_D1:
    case TEMP_CH_D2:
    case TEMP_CH_AD7294:
      if( ad7294.read_temperature(channel, temp) != AD7294_I2C_READ_OK )
        return ERROR_COMM;
      else
        return OK;
      break;
    case TEMP_CH_BOARD:
      if( ad7415.read_temperature(temp) != AD7415_I2C_READ_OK )
        return ERROR_COMM;
      else
        return OK;
      break;
    default:
      return ERROR_CHANNEL;
     
  }
    
}

    

HFAMP400_Status HFAMP400::set_current_scaling(FUSE_CHANNEL channel, float factor)
{
  if (!checkboundaries((int)channel, (int)(FUSE_CHANNEL)FUSE_CH_1, (int)(FUSE_CHANNEL)FUSE_CH_3))
    return ERROR_CHANNEL;  

  current_factor[channel] = factor;
  
  return OK;
}


HFAMP400_Status HFAMP400::set_voltage_scaling(FUSE_CHANNEL channel, float factor)
{
  if (!checkboundaries((int)channel, (int)(FUSE_CHANNEL)FUSE_CH_1, (int)(FUSE_CHANNEL)FUSE_CH_3))
    return ERROR_CHANNEL;  

  voltage_factor[channel] = factor;
  
  return OK;
}

HFAMP400_Status HFAMP400::get_fuse_current(FUSE_CHANNEL channel, float *current)
{
  int adc;
  AD7294_ADC_CHANNEL ad7294_channel;
  HFAMP400_Status status = OK;

  if (!checkboundaries((int)channel, (int)(FUSE_CHANNEL)FUSE_CH_1, (int)(FUSE_CHANNEL)FUSE_CH_3))
    return ERROR_CHANNEL;  


  if(channel == FUSE_CH_3)
  {
    adc = analogRead(A5);
  } else {
    if (channel == FUSE_CH_1)
      ad7294_channel = AD7294_ADC_CH_VIN_0;
    else 
      ad7294_channel = AD7294_ADC_CH_VIN_2;

    status = get_ad7294_adcvalue(ad7294_channel, &adc);
 
  }

#ifdef DEBUG_CURRENT
  Serial.print("current adc = ");
  Serial.println(adc);
#endif

  if (status == OK) 
    //*current = (adc - current_offset[(int)channel - FUSE_CH_1]) * current_factor[(int)channel - FUSE_CH_1];
    *current = (adc) * current_factor[(int)channel - FUSE_CH_1];
  
  return status;
}


HFAMP400_Status HFAMP400::get_fuse_voltage(FUSE_CHANNEL channel, float *voltage)
{
  int ch;

  if (!checkboundaries((int)channel, (int)(FUSE_CHANNEL)FUSE_CH_1, (int)(FUSE_CHANNEL)FUSE_CH_3))
    return ERROR_CHANNEL;  

  switch(channel)
  {
    case FUSE_CH_1:
      ch = A2;
      break;
    case FUSE_CH_2:
      ch = A3;
      break;
    case FUSE_CH_3:
      ch = A4;
      break;
   
  }


  *voltage = analogRead(ch) * voltage_factor[(int)channel - FUSE_CH_1];
    
  return OK;
}

HFAMP400_Status HFAMP400::get_hfpower(HFPOWER_CHANNEL channel, float *power)
{
  HFAMP400_Status res;
  float voltage;
  
  if (!checkboundaries((int)channel, (int)(HFPOWER_CHANNEL)HF_CH_FORWARD_1, (int)(HFPOWER_CHANNEL)HF_CH_RETURN_2))
    return ERROR_CHANNEL;

  res = get_adl5513_voltage(channel, &voltage);
  if (res != OK)
    return res;



  *power = voltage_to_hfpower(channel, voltage);
  
#ifdef DEBUG
  Serial.print("HF-Power Channel=");
  Serial.print((int)channel);
  Serial.print(" voltage= ");
  Serial.print(voltage,4);
  Serial.print(" power= ");
  Serial.println(*power,4);
#endif

  return OK;
  
}

HFAMP400_Status HFAMP400::set_working_frequency(float frequency)
{
  float scale, offset;
  int i;
  HFPOWER_CONVERSION hf_conv;
  
  working_frequency = frequency;

  for (i = HF_CH_FORWARD_1; i <= HF_CH_RETURN_2; i++) {
    hfpowercal.get_hfpower_conversion( (HFPOWER_CHANNEL)i,  frequency,  &hf_conv);
    set_hfpower_conversion( (HFPOWER_CHANNEL)i, hf_conv );
  }

  return OK;
 
}

HFAMP400_Status HFAMP400::set_hfpower_conversion(HFPOWER_CHANNEL channel, HFPOWER_CONVERSION hf_conv)
{
  if (!checkboundaries((int)channel, (int)(HFPOWER_CHANNEL)HF_CH_FORWARD_1, (int)(HFPOWER_CHANNEL)HF_CH_RETURN_2))
    return ERROR_CHANNEL;

  memcpy(&hfpower_conversion[(int)channel],&hf_conv, sizeof( HFPOWER_CONVERSION ));
  return OK;
}

HFAMP400_Status HFAMP400::set_hfpower_modulation_factor(HFPOWER_CHANNEL channel, float factor)
{
  if (!checkboundaries((int)channel, (int)(HFPOWER_CHANNEL)HF_CH_FORWARD_1, (int)(HFPOWER_CHANNEL)HF_CH_RETURN_2))
    return ERROR_CHANNEL;  

  hfpower_modulation_factor[(int)(channel-HF_CH_FORWARD_1)] = factor;

  return OK;
}

HFAMP400_Status HFAMP400::get_adl5513_voltage(HFPOWER_CHANNEL channel, float *voltage)
{

  HFAMP400_Status res=OK;
  int adc_voltage;
  
  switch(channel)
  {
    case HF_CH_FORWARD_1:
      *voltage = analogRead(A0) * 3.3 / (float)0xFFF;
      break;
    case HF_CH_FORWARD_2:
      *voltage = analogRead(A1) * 3.3 / (float)0xFFF;
      break;
    case HF_CH_RETURN_1:
      res = get_ad7294_adcvalue(AD7294_ADC_CH_VIN_1, &adc_voltage);
      *voltage = (float)adc_voltage * 2.5 / (float)0xFFF;
      break;
    case HF_CH_RETURN_2:
      res = get_ad7294_adcvalue(AD7294_ADC_CH_VIN_3, &adc_voltage);
      *voltage = (float)adc_voltage * 2.5 / (float)0xFFF;
      break;
  }

  return res;
}


bool HFAMP400::checkboundaries( float var, float min_value, float max_value)
{
  if(var >= min_value && var <=max_value) 
    return true;
  else
    return false;
}


bool HFAMP400::checkboundaries( int var, int min_value, int max_value)
{
  if(var >= min_value && var <=max_value) 
    return true;
  else
    return false;
}

int HFAMP400::read_line(char* buffer, int bufsize)
{
  for (int index = 0; index < bufsize; index++) {
    // Wait until characters are available
    while (Serial.available() == 0) {
    }

    char ch = Serial.read(); // read next character
    Serial.print(ch); // echo it back: useful with the serial monitor (optional)

    if (ch == '\n' || ch == '\r') {
      buffer[index] = 0; // end of line reached: null terminate string
      return index; // success: return length of string (zero if string is empty)
    }

    buffer[index] = ch; // Append character to buffer
  }

  // Reached end of buffer, but have not seen the end-of-line yet.
  // Discard the rest of the line (safer than returning a partial line).

  char ch;
  do {
    // Wait until characters are available
    while (Serial.available() == 0) {
    }
    ch = Serial.read(); // read next character (and discard it)
    Serial.print(ch); // echo it back
  } while (ch != '\n');

  buffer[0] = 0; // set buffer to empty string even though it should not be used
  return -1; // error: return negative one to indicate the input was too long
}




void HFAMP400::run_hfpower_calibration(void)
{
  run_hfpower_calibration(0xF);
}

void HFAMP400::run_voltage_calibration(void)
{
  int i,j,k;
  float voltage,average,factor;
  char cstring[31];
    
  set_voltage_scaling(FUSE_CH_1, HFAMP400_DEFAULT_VOLTAGE_SCALING);
  set_voltage_scaling(FUSE_CH_2, HFAMP400_DEFAULT_VOLTAGE_SCALING);
  set_voltage_scaling(FUSE_CH_3, HFAMP400_DEFAULT_VOLTAGE_SCALING);

  Serial.println("===============================");
  Serial.println("= calibration routine started =");
  Serial.println("===============================");
   

  for(i = FUSE_CH_1; i <=  FUSE_CH_3; i++)
  {
    
    Serial.print("Apply 12V to Channel ");
    Serial.println(i-FUSE_CH_1+1);
    while( Serial.available() == 0 ) {
      get_fuse_voltage((FUSE_CHANNEL)i, &voltage);
      Serial.println(voltage);
      delay(1000);
    }
    read_line(cstring, 1);

    average = 0.0;
    for(k = 0; k <= 3; k++)
    {
      get_fuse_voltage((FUSE_CHANNEL)i, &voltage);
      average += voltage;
      delay(200);
    }      

    average /= 4.0;
    Serial.print("Measured: ");
    Serial.print(average);
    Serial.print(" @ Channel ");
    Serial.println(i+1);

    factor =  12.0 / average * HFAMP400_DEFAULT_VOLTAGE_SCALING;
    
    hfpowercal.eeprom.write_buffer(EE_VOLTAGE_CALIBRATION+sizeof(float)*(i-FUSE_CH_1), (byte *)&factor,sizeof(float));
  }

  
  Serial.println("=================================");
  Serial.println("= Voltage calibration completed =");
  Serial.println("=================================");
  read_line(cstring, 1);
}


void HFAMP400::run_current_calibration(void)
{
  int i,j,k;
  float current,average,factor;
  char cstring[31];
  
  set_current_scaling(FUSE_CH_1, HFAMP400_DEFAULT_CURRENT_SCALING);
  set_current_scaling(FUSE_CH_2, HFAMP400_DEFAULT_CURRENT_SCALING);
      

  Serial.println("===============================");
  Serial.println("= calibration routine started =");
  Serial.println("===============================");
   

  for(i = FUSE_CH_1; i <=  FUSE_CH_2; i++)
  {
     
    Serial.print("Apply 2A to Channel ");
    Serial.println(i-FUSE_CH_1+1);
    while( Serial.available() == 0 ) {
      get_fuse_current((FUSE_CHANNEL)i, &current);
      Serial.println(current);
      delay(1000);
    }
    read_line(cstring, 1);

    average = 0.0;
    for(k = 0; k <= 3; k++)
    {
      get_fuse_current((FUSE_CHANNEL)i, &current);
      average += current;
      delay(200);
    }      

    average /= 4.0;
    Serial.print("Measured: ");
    Serial.print(average);
    Serial.print(" @ Channel ");
    Serial.println(i+1);

    factor = 2.0 / average * HFAMP400_DEFAULT_CURRENT_SCALING;
    
    hfpowercal.eeprom.write_buffer(EE_CURRENT_CALIBRATION+sizeof(float)*(i-FUSE_CH_1), (byte*)&factor,sizeof(float));
  }

  
  Serial.println("=================================");
  Serial.println("= Current calibration completed =");
  Serial.println("=================================");
  read_line(cstring, 1);
}



void HFAMP400::run_hfpower_calibration(uint8_t mask)
{
  int i,j,k;
  int takepoints;
  char cstring[31];
  float voltage,voltage_neg20dBm[MAX_CAL_POINTS],voltage_neg40dBm[MAX_CAL_POINTS],freqs[MAX_CAL_POINTS],curr_freq;
  String calid;
  
  
  takepoints = (int)(sizeof(hfpowercal.std_frequency_table)/sizeof(float));
  hfpowercal.write_num_points(takepoints);
  
  Serial.println("===============================");
  Serial.println("= calibration routine started =");
  Serial.println("===============================");
  Serial.print("configured for ");
  Serial.print(takepoints);
  Serial.println(" calibration points per channel");
  Serial.print("Enter calibration id-string: ");


  read_line(cstring, 30);
  if(cstring[0])
    hfpowercal.write_cal_string(cstring);
    
#ifdef DEBUG
  Serial.println("");
  Serial.println(mask,HEX);
  Serial.println("");
#endif      

  for(i = HF_CH_FORWARD_1; i <=  HF_CH_RETURN_2; i++)
  {
    if (!(mask & bit(i))) continue;
    
    Serial.print("Connect HF-Generator with to Channel ");
    Serial.println(i);
    while( Serial.available() == 0 ) {
      get_adl5513_voltage((HFPOWER_CHANNEL)i, &voltage);
      Serial.println(voltage);
      delay(1000);
    }
    read_line(cstring, 1);
    
    for( j = 0; j < takepoints; j++)
    {
      freqs[j] = curr_freq = hfpowercal.std_frequency_table[j];
      Serial.print("Set Frequency to ");
      Serial.print(curr_freq/1e6);
      Serial.println("Mhz and Power to -20dBm");
      Serial.println("Press ENTER when ready");
      read_line(cstring, 1);

      voltage_neg20dBm[j] = 0.0;
      for(k = 0; k <= 3; k++)
      {
        get_adl5513_voltage((HFPOWER_CHANNEL)i, &voltage);
        voltage_neg20dBm[j] += voltage;
        delay(200);
      }      
      
      voltage_neg20dBm[j] /= 4.0;
   
      Serial.print("Measured: ");
      Serial.print(voltage_neg20dBm[j]);
      Serial.print(" @ ");
      Serial.print(curr_freq/1e6);
      Serial.println("Mhz (-20dBm)");
    }
    
    for( j = 0; j < takepoints; j++)
    {    
      curr_freq = hfpowercal.std_frequency_table[j];
      Serial.print("Set Frequency to ");
      Serial.print(curr_freq/1e6);
      Serial.println("Mhz and Power to -40dBm");
      Serial.println("Press ENTER when ready");
      read_line(cstring, 1);

      voltage_neg40dBm[j] = 0.0;
      for(k = 0; k <= 3; k++)
      {
        get_adl5513_voltage((HFPOWER_CHANNEL)i, &voltage);
        voltage_neg40dBm[j] += voltage;
        delay(200);
      }      
      
      voltage_neg40dBm[j] /= 4.0;
      
      Serial.print("Measured: ");
      Serial.print(voltage_neg40dBm[j]);
      Serial.print(" @ ");
      Serial.print(curr_freq/1e6);
      Serial.println("Mhz (-40dBm)");


    }
    
    for( j = 0; j < takepoints; j++)
    {    
      
      hfpowercal.write_point(j,(HFPOWER_CHANNEL)i, freqs[j], voltage_neg20dBm[j],voltage_neg40dBm[j]);
    }
  } 
  
  Serial.println("=========================");
  Serial.println("= calibration completed =");
  Serial.println("=========================");
  read_line(cstring, 1);
}


HFAMP400_Status HFAMP400::set_alert_limit(ALERT_LIMIT limit, float max_value)
{ 
  float min_value = 0;

  
  switch(limit)
  {
    case ALERT_TEMP_D1:
    case ALERT_TEMP_D2:
    case ALERT_TEMP_AD7294:
      min_value = -40.0; //deg C
      break;
    case ALERT_HFPOWER_RETURN_1:
    case ALERT_HFPOWER_RETURN_2:
      min_value = -80.0; //dBm
      break;
    case ALERT_CURRENT_FUSE_1:      
    case ALERT_CURRENT_FUSE_2:
      min_value = 0.0; // Ampere        
      break;  
  }

  
  return set_alert_limit(limit, min_value, max_value);
}

HFAMP400_Status HFAMP400::set_alert_limit(ALERT_LIMIT limit, float min_value, float max_value)
{
  uint16_t conv_min, conv_max,hyst;
  AD7294_Limits ad7294_limit;
  AD7294_Status res;
  
  
  switch(limit)
  {
    case ALERT_TEMP_D1:
    case ALERT_TEMP_D2:
    case ALERT_TEMP_AD7294:
      conv_min = ad7294.temp_float_to_limitreg(min_value);
      conv_max = ad7294.temp_float_to_limitreg(max_value);
      hyst = 0x3FE;
      break;
    case ALERT_HFPOWER_RETURN_1:
      conv_min = hfpower_float_to_limitreg(HF_CH_RETURN_1,min_value);
      conv_max = hfpower_float_to_limitreg(HF_CH_RETURN_1,max_value);
      hyst = 0xFFE;
#ifdef DEBUG
      Serial.print("HF-Limit min= ");
      Serial.print(conv_min);
      Serial.print(" max= ");
      Serial.println(conv_max);
#endif
      break;
    case ALERT_HFPOWER_RETURN_2:
      conv_min = hfpower_float_to_limitreg(HF_CH_RETURN_2,min_value);
      conv_max = hfpower_float_to_limitreg(HF_CH_RETURN_2,max_value);
      hyst = 0xFFE;
      break;
    case ALERT_CURRENT_FUSE_1:
      conv_min = (1/current_factor[FUSE_CH_1]) * min_value;
      conv_max = (1/current_factor[FUSE_CH_1]) * max_value;
      hyst = 0xFFE;
      break;
      
    case ALERT_CURRENT_FUSE_2:
      conv_min = (1/current_factor[FUSE_CH_2]) * min_value;
      conv_max = (1/current_factor[FUSE_CH_2]) * max_value;
      hyst = 0xFFE;
      break;        
  }

  switch(limit)
  {
    case ALERT_TEMP_D1:
      ad7294_limit = AD7294_LIMIT_TSENSE_1;
      break;
    case ALERT_TEMP_D2:
      ad7294_limit = AD7294_LIMIT_TSENSE_2;
      break;
    case ALERT_TEMP_AD7294:
      ad7294_limit = AD7294_LIMIT_TSENSE_INT;
      break;
    case ALERT_HFPOWER_RETURN_1:
      ad7294_limit = AD7294_LIMIT_VIN_1;
      break;
    case ALERT_HFPOWER_RETURN_2:
      ad7294_limit = AD7294_LIMIT_VIN_3;
      break;
    case ALERT_CURRENT_FUSE_1:
      ad7294_limit = AD7294_LIMIT_VIN_0;
      break;    
    case ALERT_CURRENT_FUSE_2:
      ad7294_limit = AD7294_LIMIT_VIN_2;
      break;   
  }

#ifdef DEBUG
   Serial.print("Limit(min.) converted ");
   Serial.println((int)conv_min,HEX);
   Serial.print("Limit(max.) converted ");
   Serial.println((int)conv_max,HEX);
#endif

   res=ad7294.set_limit( ad7294_limit, conv_min, conv_max, hyst);

   if(res != AD7294_I2C_WRITE_OK)
    return ERROR_COMM;

  return OK;
}


float HFAMP400::voltage_to_hfpower(HFPOWER_CHANNEL channel,float voltage)
{
  float power;
  
  power = (voltage-hfpower_conversion[(int)channel].offset_neg40dBm)/hfpower_conversion[(int)channel].scale - 40.0; 
  power *= hfpower_modulation_factor[(int)channel]; //apply correction factor

  return power;
  
}

uint16_t HFAMP400::hfpower_float_to_limitreg(HFPOWER_CHANNEL channel,float power)
{
  float res;

  //ATTENTION: modulation factor is not taken into consideration 
  res = (power + 40.0) * hfpower_conversion[(int)channel].scale + hfpower_conversion[(int)channel].offset_neg40dBm;

  return (res / 2.5) * 0xFFF;
}

HFAMP400 hfamp400;


