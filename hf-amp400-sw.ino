
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
#include <SPI.h>
#include <Ethernet.h>


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 0, 117);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(4711);

const int led_pin = 13;

uint32_t num_alerts=0;

#define SERIALPRINT_INTERVAL 1000 //ms
#define ADCREAD_INTERVAL 500 //ms


int last_serialprint = millis() - SERIALPRINT_INTERVAL;
int last_adcread = millis() - ADCREAD_INTERVAL;


void alert_interrupt()
{
  //Don't use I2C communication in interrupt!!!
  //Keep it short
  //Don't use delay ()
  //Don't do serial prints
  //Make variables shared with the main code volatile
  //Variables shared with main code may need to be protected by "critical sections" (see below)
  //Don't try to turn interrupts off or on
  //millis() wont increment

  num_alerts++;
}

void setup() {
  EE24LC64 eui48eeprom;
  char response[3];
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  

  if(eui48eeprom.init(0x51,false) == 0)
  {
    Serial.println("Found EUI48 EEPROM");
    eui48eeprom.read_buffer(0xFA,mac,6);
  }

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();

  pinMode(led_pin, OUTPUT);
  
  hfamp400.init();

  //Alert Interrupt Example
  attachInterrupt(ALERT_PIN, alert_interrupt, FALLING);  //signla line 

  //Board specific tuning of conversion factors - OPTIONAL - determined empirically
  hfamp400.set_voltage_scaling(FUSE_CH_1, HFAMP400_DEFAULT_VOLTAGE_SCALING*1.0326);
  hfamp400.set_voltage_scaling(FUSE_CH_2, HFAMP400_DEFAULT_VOLTAGE_SCALING*1.0326);
  hfamp400.set_voltage_scaling(FUSE_CH_3, HFAMP400_DEFAULT_VOLTAGE_SCALING*1.0326);
  
  hfamp400.set_current_scaling(FUSE_CH_1, HFAMP400_DEFAULT_CURRENT_SCALING*4.00/3.43); 
  hfamp400.set_current_scaling(FUSE_CH_2, HFAMP400_DEFAULT_CURRENT_SCALING*4.00/3.80);   
  


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
    Serial.println("starting hf calibration ...");
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

  Serial.println("Run Voltage Calibration [y/N]?");
  hfamp400.read_line(response,2);
  if( response[0] == 'y' || response[0] == 'Y' )
  {
    hfamp400.set_voltage_scaling(FUSE_CH_1, HFAMP400_DEFAULT_VOLTAGE_SCALING);
    hfamp400.set_voltage_scaling(FUSE_CH_2, HFAMP400_DEFAULT_VOLTAGE_SCALING);
    hfamp400.set_voltage_scaling(FUSE_CH_3, HFAMP400_DEFAULT_VOLTAGE_SCALING);

     
    Serial.println("starting voltage calibration ...");
    hfamp400.run_voltage_calibration();
  }

  Serial.println("Run Current Calibration [y/N]?");
  hfamp400.read_line(response,2);
  if( response[0] == 'y' || response[0] == 'Y' )
  {
    hfamp400.set_current_scaling(FUSE_CH_1, HFAMP400_DEFAULT_CURRENT_SCALING);
    hfamp400.set_current_scaling(FUSE_CH_2, HFAMP400_DEFAULT_CURRENT_SCALING);
      
    Serial.println("starting current calibration ...");
    //hfamp400.run_current_calibration();
  }


  

  //Reset the Fuses  
  //    ############### fuse2 or fuse3 selected by jumper 
  hfamp400.reset_fuse(FUSE_CH_1);
  hfamp400.reset_fuse(FUSE_CH_2);
//  hfamp400.reset_fuse(FUSE_CH_3); wurks but same as FUSE_CH_2
}

void loop() {
  uint32_t alerts;
  char c;
  static float temp[4];
  static float power[4];
  static float current[3]; // fuse 
  static float voltage[3]; // fuse  after FET  to control proper release of fuse , also usable for dc POWER INPUT MEASUREMENTS

 

  if((last_adcread + ADCREAD_INTERVAL) <= millis())
  {
    //Read Temperatures
    
    hfamp400.get_temperature(TEMP_CH_D1,&temp[0]);
    hfamp400.get_temperature(TEMP_CH_D2,&temp[1]);
    hfamp400.get_temperature(TEMP_CH_AD7294,&temp[2]);
    hfamp400.get_temperature(TEMP_CH_BOARD,&temp[3]);

    //Read HF-Power
    
    hfamp400.get_hfpower(HF_CH_FORWARD_1,&power[0]);
    hfamp400.get_hfpower(HF_CH_FORWARD_2,&power[1]);
    hfamp400.get_hfpower(HF_CH_RETURN_1,&power[2]);
    hfamp400.get_hfpower(HF_CH_RETURN_2,&power[3]);
  
  
    //Read Voltage
    
    hfamp400.get_fuse_voltage(FUSE_CH_1, &voltage[0]);
    hfamp400.get_fuse_voltage(FUSE_CH_2, &voltage[1]);
    hfamp400.get_fuse_voltage(FUSE_CH_3, &voltage[2]);
    
  
    //Read Current
    
    hfamp400.get_fuse_current(FUSE_CH_1, &current[0]);
    hfamp400.get_fuse_current(FUSE_CH_2, &current[1]);
    hfamp400.get_fuse_current(FUSE_CH_3, &current[2]);
    
    last_adcread=millis();
  }   
  
  if((last_serialprint + SERIALPRINT_INTERVAL) <= millis())
  {
    Serial.println("");
    
    //Print Temperatures
    Serial.print("Temperatures D1= ");
    Serial.print(temp[0]);
    Serial.print(" D2= ");
    Serial.print(temp[1]);
    Serial.print(" AD7294= ");
    Serial.print(temp[2]);
    Serial.print(" BOARD= ");
    Serial.print(temp[3]);
    Serial.println(" [deg C]");

    //Print HF-Power
    Serial.print("HFPower Forward_1= ");
    Serial.print(power[0]);
    Serial.print(" Forward_2= ");
    Serial.print(power[1]);
    Serial.print(" Return_1= ");
    Serial.print(power[2]);
    Serial.print(" Return_2= ");
    Serial.print(power[3]);
    Serial.println(" [dBm]");

    //Print Fuse Currents

    Serial.print("Fuse-Voltage CH1= ");
    Serial.print(voltage[0]);
    Serial.print(" CH2= ");
    Serial.print(voltage[1]);
    Serial.print(" CH3= ");
    Serial.print(voltage[2]);
    Serial.println(" [V]");

    //Print Fuse Voltages

    Serial.print("Fuse-Current CH1= ");
    Serial.print(current[0]);
    Serial.print(" CH2= ");
    Serial.print(current[1]);
    Serial.print(" CH3= ");
    Serial.print(current[2]);
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

    hfamp400.ad7294.clear_alerts(); //alerts needs to be cleared explicitly  

    last_serialprint = millis();
  }
 

    // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
   
    if (client.connected()) {
      if(client.available())
      {
        c = client.read();
        if(c == '1') digitalWrite(led_pin, HIGH);   // turn the LED on (HIGH is the voltage level)
        if(c == '0') digitalWrite(led_pin, LOW);  
        if(c == 'r') {
          client.print(temp[0]);
          client.print(",");
          client.print(temp[1]);
          client.print(",");
          client.print(temp[2]);
          client.print(",");
          client.print(temp[3]);
          client.print((char)0x0A);
          
        }
      }
    } else {
      client.stop();
    }
  } 
   
}

