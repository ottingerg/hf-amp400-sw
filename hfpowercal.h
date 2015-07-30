/*
* hfpowercal.h - Routines for Calibrating the ADL5513s on HFAmp400
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

#ifndef HFPOWERCAL_H_INCLUDED
#define HFPOWERCAL_H_INCLUDED


#include "Arduino.h"
#include "ee24lc64.h"

#define MAX_CAL_POINTS 50
//#define SMALL_FREQ_TABLE

struct {
  float scale;
  float offset_neg40dBm;
} typedef HFPOWER_CONVERSION;

enum HFPOWERCAL_Status
{
  HFPOWER_CONV_VALID,
  ERROR_FREQ_RANGE,
};

enum HFPOWER_CHANNEL 
{
  HF_CH_FORWARD_1 = 0,
  HF_CH_FORWARD_2 = 1,
  HF_CH_RETURN_1 = 2,
  HF_CH_RETURN_2 = 3,
};


/* EEPROM-Map
   0x00 - 0xFF Reserved (EUI-48 for Mac, etc...)
   0x100 - 0x11E Calibration String (30)
   0x11E - 0x11E Number of Points for Calibration (uint8_t 0 .. 255 Points)
   0x11F - 0x11F Reserved
   0x120 - 0x123 Frequency of 1st Calibration Point for Channel 1 (represented as float)
   0x124 - 0x127 Voltage of 1st Calibration Point for Channel 1 (represented as float)
   0x124 - 0x127 -40dBm Voltage of 1st Calibration Point for Channel 1 (represented as float)
   0x127 - 0x12A -20dBm Voltage of 1st Calibration Point for Channel 1 (represented as float)
   
   3-Tupels of Calibration Points reapeat till number of Points for Calibration is reached.
   than follows the same amount of 3-Tupels for Channel 2, Channel 3 and Channel 4
 ...
 ...

Also note that the two times 2-Tuples are respecting the 32 Bytes page boundary
 
 */

#define EE_CAL_STRING 0x100
#define EE_NUM_POINTS 0x11E
#define EE_POINT_OFFSET 0x120



class HFPOWERCAL
{
public:
#ifdef SMALL_FREQ_TABLE
  const float std_frequency_table[3] = { 
    1.0e6, 10.0e6, 2800.0e6 };
#else
  const float std_frequency_table[19] = { 
    1.0e6, 10.0e6, 100.0e6, 200.0e6, 300.0e6, 
    400.0e6, 500.0e6, 600.0e6, 700.0e6, 
    800.00e6, 900.0e6, 1000.0e6, 1100.0e6, 
    1200.0e6, 1500.0e6, 2000.0e6, 2500.0e6, 
    3000.0e6, 3500.0e6 };
#endif  

  EE24LC64 eeprom;
  void init ();

  void write_cal_string(char *str);
  char *read_cal_string(void);
  void write_num_points(uint8_t num);
  void write_point (uint8_t point_nr, HFPOWER_CHANNEL channel, float freq, float voltage_neg20dBm, float voltage_neg40dBm);
  void export_csv(void);
  HFPOWERCAL_Status get_hfpower_conversion(HFPOWER_CHANNEL channel, float frequency, HFPOWER_CONVERSION *hf_conv);
  

    
private:
  
  uint8_t numpoints=100;
  char cal_string[31];
  
  int calc_3tupel_offset( uint8_t point_nr, HFPOWER_CHANNEL channel );
  int get_cal_point(float frequency);
};

#endif				// HFPOWERCAL_H_INCLUDED
