
/*
* hfamp400.ino - HFAmp400 Demo Example
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

void alert_interrupt()
{
 // interruot service routine
  Serial.print("Alert recieved!");
  //Don't use I2C communication in interrupt!!!
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  char response[3];
  hfamp400.init();

  //Alert Interrupt Example
  attachInterrupt(ALERT_PIN, alert_interrupt, FALLING);  //signla line 

  //Board specific tuning of conversion factors - OPTIONAL - determined empirically
  hfamp400.set_voltage_scaling(FUSE_CH_1, HFAMP400_DEFAULT_VOLTAGE_SCALING*1.0326);
  hfamp400.set_voltage_scaling(FUSE_CH_2, HFAMP400_DEFAULT_VOLTAGE_SCALING*1.0326);
  hfamp400.set_voltage_scaling(FUSE_CH_3, HFAMP400_DEFAULT_VOLTAGE_SCALING*1.0326);
  
  //hfamp400.set_current_scaling(FUSE_CH_1, HFAMP400_DEFAULT_CURRENT_SCALING*4.00/4.52); 
  //hfamp400.set_current_scaling(FUSE_CH_2, HFAMP400_DEFAULT_CURRENT_SCALING*4.00/3.75);   
  


  // GateBias Example
  //#### Marked on Board with Vout A ... Vout D
  //Set GateBias Channel A to 1.5V
  hfamp400.set_gatebias(GATEBIAS_CH_VOUT_A, 1.5);
  //Set GateBias Channel B to 2.2V
  hfamp400.set_gatebias(GATEBIAS_CH_VOUT_B, 2.2);
  //Set GateBias Channel C to 2.7V
  hfamp400.set_gatebias(GATEBIAS_CH_VOUT_C, 2.7);
  //Set GateBias Channel D to 3.21V
  hfamp400.set_gatebias(GATEBIAS_CH_VOUT_D, 3.21);

  hfamp400.set_working_frequency(100e6); //Set Working Frequency to 100 Mhz => adjusts the hfpower cal. factors
  // default 100MHz 
  // [1MHz...3.5GHz]    Setting range 
  

  //Set Limits
  hfamp400.set_alert_limit(ALERT_TEMP_D1, 33.00); //Set Alert Limit of D1 to 34.50 degrees
  hfamp400.set_alert_limit(ALERT_TEMP_D2, 33.00);
  hfamp400.set_alert_limit(ALERT_TEMP_AD7294, 70.00);
  hfamp400.set_alert_limit(ALERT_CURRENT_FUSE_1, 3.0); //Set Alert Limit of Fuse 1 Current to 2.5 Amps
  hfamp400.set_alert_limit(ALERT_CURRENT_FUSE_2, 3.0);
  hfamp400.set_alert_limit(ALERT_HFPOWER_RETURN_1, -12.0); //Ser Alert Limit of HFPOWER Return 1 to -12 dBm
  hfamp400.set_alert_limit(ALERT_HFPOWER_RETURN_2, -12.0);


  //Set Modulation correction Factor
  hfamp400.set_hfpower_modulation_factor(HF_CH_FORWARD_1, 1.0);
  hfamp400.set_hfpower_modulation_factor(HF_CH_FORWARD_2, 1.0);
  hfamp400.set_hfpower_modulation_factor(HF_CH_RETURN_1, 1.0);
  hfamp400.set_hfpower_modulation_factor(HF_CH_RETURN_2, 1.0);


  
  Serial.println("Run HF Calibration [y/N]?");
  hfamp400.read_line(response,2);
  if( response[0] == 'y' || response[0] == 'Y' )
  {
    Serial.println("starting calibration ...");
    hfamp400.run_hfpower_calibration();
  }

/*
  
  Serial.println("Run Current Offset Calibration [y/N]?");
  hfamp400.read_line(response,2);
  if( response[0] == 'y' || response[0] == 'Y' )
  {
    Serial.println("starting calibration ...");
    hfamp400.run_current_offset_calibration();
  }
*/
  
  Serial.println("Export HF-Cal Data [y/N]?");
  hfamp400.read_line(response,2);
  if( response[0] == 'y' || response[0] == 'Y' )
  {
    hfamp400.hfpowercal.export_csv();
    delay(10000);
  }

  

  //Reset the Fuses  
  //    ############### fuse2 or fuse3 selected by jumper 
  hfamp400.reset_fuse(FUSE_CH_1);
  hfamp400.reset_fuse(FUSE_CH_2);
//  hfamp400.reset_fuse(FUSE_CH_3); wurks but same as FUSE_CH_2
}

void loop() {
  uint32_t alerts;
  float temp[4];
  float power[4];
  float current[3]; // fuse 
  float voltage[3]; // fuse  after FET  to control proper release of fuse , also usable for dc POWER INPUT MEASUREMENTS


  Serial.println("");
  
  //Read Temperatures
  
  hfamp400.get_temperature(TEMP_CH_D1,&temp[0]);
  hfamp400.get_temperature(TEMP_CH_D2,&temp[1]);
  hfamp400.get_temperature(TEMP_CH_AD7294,&temp[2]);
  hfamp400.get_temperature(TEMP_CH_BOARD,&temp[3]);

  Serial.print("Temperatures D1= ");
  Serial.print(temp[0]);
  Serial.print(" D2= ");
  Serial.print(temp[1]);
  Serial.print(" AD7294= ");
  Serial.print(temp[2]);
  Serial.print(" BOARD= ");
  Serial.print(temp[3]);
  Serial.println(" [deg C]");
 
  //Read HF-Power
  
  hfamp400.get_hfpower(HF_CH_FORWARD_1,&power[0]);
  hfamp400.get_hfpower(HF_CH_FORWARD_2,&power[1]);
  hfamp400.get_hfpower(HF_CH_RETURN_1,&power[2]);
  hfamp400.get_hfpower(HF_CH_RETURN_2,&power[3]);

  Serial.print("HFPower Forward_1= ");
  Serial.print(power[0]);
  Serial.print(" Forward_2= ");
  Serial.print(power[1]);
  Serial.print(" Return_1= ");
  Serial.print(power[2]);
  Serial.print(" Return_2= ");
  Serial.print(power[3]);
  Serial.println(" [dBm]");


  //Read Voltage
  
  hfamp400.get_fuse_voltage(FUSE_CH_1, &voltage[0]);
  hfamp400.get_fuse_voltage(FUSE_CH_2, &voltage[1]);
  hfamp400.get_fuse_voltage(FUSE_CH_3, &voltage[2]);
  
  Serial.print("Fuse-Voltage CH1= ");
  Serial.print(voltage[0]);
  Serial.print(" CH2= ");
  Serial.print(voltage[1]);
  Serial.print(" CH3= ");
  Serial.print(voltage[2]);
  Serial.println(" [V]");

  //Read Current
  
  hfamp400.get_fuse_current(FUSE_CH_1, &current[0]);
  hfamp400.get_fuse_current(FUSE_CH_2, &current[1]);
  hfamp400.get_fuse_current(FUSE_CH_3, &current[2]);
  
  Serial.print("Fuse-Current CH1= ");
  Serial.print(current[0]);
  Serial.print(" CH2= ");
  Serial.print(current[1]);
  Serial.print(" CH3= ");
  Serial.print(current[3]);
  Serial.println(" [A]");

  
  alerts = hfamp400.ad7294.get_alerts();
  
  if(alerts & ALERT_MASK_TEMP_D1)
    Serial.println("!!! ALERT: TEMP_D1");
  if(alerts & ALERT_MASK_TEMP_D2)
    Serial.println("!!! ALERT: TEMP_D2");
  if(alerts & ALERT_MASK_TEMP_INT)
    Serial.println("!!! ALERT: TEMP_INT");
  if(alerts & ALERT_MASK_HFPOWER_RETURN_1)
    Serial.println("!!! ALERT: HFPOWER_RETURN_1");
  if(alerts & ALERT_MASK_HFPOWER_RETURN_2)
    Serial.println("!!! ALERT: HFPOWER_RETURN_2");
  if(alerts & ALERT_MASK_FUSE_1)
    Serial.println("!!! ALERT: FUSE_1");
  if(alerts & ALERT_MASK_FUSE_2)
    Serial.println("!!! ALERT: FUSE_2");

  
  
  delay(1000);
      
  hfamp400.ad7294.clear_alerts(); //alerts needs to be cleared explicitly  
  
   
}
