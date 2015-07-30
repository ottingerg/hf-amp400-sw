/*
* hfpowercal.cpp - Routines for Calibrating the ADL5513s on HFAmp400
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

#include "hfpowercal.h"

//#define DEBUG

void HFPOWERCAL::init ()
{
   eeprom.init();
   cal_string[30] = 0; //Terminate Cal-String
   read_cal_string();
   numpoints=eeprom.read_byte(EE_NUM_POINTS);

#ifdef DEBUG
  Serial.print("HFPOWERCAL: Calibration String: ");
  Serial.println(cal_string);
  Serial.print("HFPOWERCAL: Number of Points: ");
  Serial.println(numpoints);
#endif

}

char *HFPOWERCAL::read_cal_string(void)
{
  eeprom.read_buffer(EE_CAL_STRING, (byte*)cal_string, 30);

  return cal_string;
}

void HFPOWERCAL::write_cal_string(char *str)
{
  strncpy(cal_string,str,30);
  eeprom.write_buffer(EE_CAL_STRING,(byte*)cal_string,30);
}

void HFPOWERCAL::write_num_points(uint8_t num)
{
  numpoints = num;
  eeprom.write_byte(EE_NUM_POINTS, (byte)num);
}

void HFPOWERCAL::write_point (uint8_t point_nr, HFPOWER_CHANNEL channel, float freq, float voltage_neg20dBm, float voltage_neg40dBm)
{
  int addr;

  addr = calc_3tupel_offset(point_nr, channel);

  eeprom.write_buffer(addr, (byte*)&freq, sizeof(float));
  addr += sizeof(float);
  eeprom.write_buffer(addr, (byte*)&voltage_neg20dBm, sizeof(float));
  addr += sizeof(float);
  eeprom.write_buffer(addr, (byte*)&voltage_neg40dBm, sizeof(float)); 
}

void HFPOWERCAL::export_csv(void)
{
  int i,j; 
  int addr;
  float temp;

  Serial.println(cal_string);
  Serial.println("\"Channel\";\"Frequency\";\"Voltage -20dBm\";\"Voltage -40dBm\"");

  for(i = HF_CH_FORWARD_1; i <= HF_CH_RETURN_2; i++)
  {
    for(j = 0; j < numpoints; j++)
    {
    

       Serial.print(i);
       Serial.print(";");
       addr = calc_3tupel_offset(j,(HFPOWER_CHANNEL)i);
       eeprom.read_buffer(addr, (byte*)&temp, sizeof(float));
       Serial.print(temp);
       Serial.print(";");
       addr += sizeof(float);
       eeprom.read_buffer(addr, (byte*)&temp, sizeof(float));
       Serial.print(temp,4);      
       Serial.print(";");
       addr += sizeof(float);
       eeprom.read_buffer(addr, (byte*)&temp, sizeof(float));
       Serial.println(temp,4);   
    }
  }
}

int HFPOWERCAL::calc_3tupel_offset( uint8_t point_nr, HFPOWER_CHANNEL channel ) 
{ 
  return ((channel-HF_CH_FORWARD_1)*numpoints+point_nr)*(sizeof(float)*3)+EE_POINT_OFFSET; 
}

int HFPOWERCAL::get_cal_point(float frequency)
{
  int j;
  int addr;
  float p1,p2;

  addr = calc_3tupel_offset(0,HF_CH_FORWARD_1);
  eeprom.read_buffer(addr, (byte*)&p1, sizeof(float));
     
  for(j = 1; j < numpoints; j++)
  {
     addr = calc_3tupel_offset(j,HF_CH_FORWARD_1);
     eeprom.read_buffer(addr, (byte*)&p2, sizeof(float));
     if (p2 >= frequency && p1 <= frequency) 
        return j-1;
     p1 = p2;  
  }

  return -1; // not found
}
  
HFPOWERCAL_Status HFPOWERCAL::get_hfpower_conversion(HFPOWER_CHANNEL channel, float frequency, HFPOWER_CONVERSION *hf_conv)
{
    int refpoint;
    int addr;
    float freq[2],neg20dBm[2], neg40dBm[2];
    float neg20dBm_interpolated;
    float neg40dBm_interpolated;
    

    refpoint = get_cal_point(frequency);
#ifdef DEBUG
      Serial.print("cal point: ");
      Serial.print(refpoint);
      Serial.print(" @ ");
      Serial.print(frequency);
      Serial.println("Mhz");
#endif
    if (refpoint < 0) 
      return ERROR_FREQ_RANGE;


   addr = calc_3tupel_offset(refpoint,channel);
   eeprom.read_buffer(addr, (byte*)&freq[0], sizeof(float));
   addr +=sizeof(float);
   eeprom.read_buffer(addr, (byte*)&neg20dBm[0], sizeof(float));
   addr +=sizeof(float);
   eeprom.read_buffer(addr, (byte*)&neg40dBm[0], sizeof(float));
   addr +=sizeof(float);
   eeprom.read_buffer(addr, (byte*)&freq[1], sizeof(float));
   addr +=sizeof(float);
   eeprom.read_buffer(addr, (byte*)&neg20dBm[1], sizeof(float));
   addr +=sizeof(float);
   eeprom.read_buffer(addr, (byte*)&neg40dBm[1], sizeof(float));

#ifdef DEBUG
  Serial.println(freq[0]);
  Serial.println(neg20dBm[0]);
  Serial.println(neg40dBm[0]);
  Serial.println(freq[1]);
  Serial.println(neg20dBm[1]);
  Serial.println(neg40dBm[1]);
#endif
  

   neg20dBm_interpolated = neg20dBm[0] + (neg20dBm[1]-neg20dBm[0])/(freq[1]-freq[0])*(frequency-freq[0]);
   neg40dBm_interpolated = neg40dBm[0] + (neg40dBm[1]-neg40dBm[0])/(freq[1]-freq[0])*(frequency-freq[0]);

#ifdef DEBUG
  Serial.print("-20dBm int ");
  Serial.println(neg20dBm_interpolated);
  Serial.print("-40dBm int ");
  Serial.println(neg40dBm_interpolated);
#endif
   
   hf_conv->scale = (neg20dBm_interpolated-neg40dBm_interpolated)/20.0;
   hf_conv->offset_neg40dBm = neg40dBm_interpolated;

#ifdef DEBUG
   Serial.print("HFCONV: freq=");
   Serial.print(frequency);
   Serial.print(" scale= ");
   Serial.print(hf_conv->scale,6);
   Serial.print(" offset= ");
   Serial.println(hf_conv->offset_neg40dBm);
#endif

   return HFPOWER_CONV_VALID;

    
}





