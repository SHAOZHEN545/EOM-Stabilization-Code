//! @todo Review this file?  Document with Doxygen?  Time permitting...
/*
DC590B USB to Serial Controller

This file contains the routines to emulate the DC590B USB to Serial Converter. All commands
are supported except Uxxy the Write Port D bus. Added the 'D' delay ms command.
With this program, the Linduino can be used by the QuikEval program running on a PC
to communicate with QuikEval compatible demo boards.

The Kxy bit bang command uses the following pin mappings :
0-Linduino 2
1-Linduino 3
2-Linduino 4
3-Linduino 5
4-Linduino 6
5-Linduino 7


Copyright 2021(c) Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 - Neither the name of Analog Devices, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 - The use of this software may or may not infringe the patent rights
   of one or more patent holders.  This license does not release you
   from the requirement that you obtain separate licenses from these
   patent holders to use this software.
 - Use of the software either in source or binary form, must be run
   on or directly connected to an Analog Devices Inc. component.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "QuikEval_EEPROM.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LT_I2C.h"
#include "LT8722.h"
#include <Wire.h>
#include <SPI.h>


// timeouts
#define READ_TIMEOUT  20
#define MISO_TIMEOUT  1000

// reference resistance
#define R_REF   1002.5  // reference resistance
#define AVERAGES  30   // range 1: 100

// Mode prameters
String serData = "";
String set_point; //= "sptemp1081";
int freq_update = 0;    // The variable indicating whether we set a new scan frequency
int M0_switch = 2;   // The variable choosing output style in M0 mode
String scan_freq_command;


// recording mode constants
#define RECORDING_SIZE 50
const byte off = 0;
const byte playback = 1;

// serial mode constants
const byte spi_mode = 0;
const byte i2c_mode = 1;
const byte i2c_auxiliary_mode = 2;

// hex conversion constants
char hex_digits[16]=
{
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

// global variables
byte serial_mode = spi_mode;  // current serial mode
byte recording_mode = off;        // recording mode off
float PID_error_t = 0;
float PID_error_p = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
char mode;      // = '0';

////////////////////////////////////////////////////
// CHANGED MAJOR VERSION to 2 FOR ENHANCED VERSION//
////////////////////////////////////////////////////
char id_string[51]="USBSPI,PIC,02,01,DC,DC590,----------------------\n\0"; // id string
char hex_to_byte_buffer[5]=
{
  '0', 'x', '0', '0', '\0'
};               // buffer for ASCII hex to byte conversion
char byte_to_hex_buffer[3]=
{
  '\0','\0','\0'
};                     // buffer for byte to ASCII hex conversion
char recording_buffer[RECORDING_SIZE]=
{
  '\0'
}; // buffer for saving recording loop
byte recording_index = 0;                // index to the recording buffer

uint8_t w_data_8[5];
uint8_t r_data_8[8];
float tempFloat = 0;
int analogPin = 4;
uint32_t i_soft = 0;

char get_char();

void output_voltage_dev(float output_voltage)
{
        typedef union
        {
          uint8_t   data_8[4];
          uint32_t  data_32;
          int32_t   data_32s;
        }dac_data;

      dac_data dac_data0;
           
      char tmp[8];
      //float writeVoltage=1.0;
      float float_temp=0.0;
      float_temp=output_voltage/16/1.25*16777216;
      dac_data0.data_32s=(int32_t)float_temp;
      
      w_data_8[0] =       0x4;           //SPI Address
      w_data_8[1] = dac_data0.data_8[3]; //WD3
      w_data_8[2] = dac_data0.data_8[2]; //WD2
      w_data_8[3] = dac_data0.data_8[1]; //WD1
      w_data_8[4] = dac_data0.data_8[0]; //WD0
      
      transfer_8722(DW, w_data_8, r_data_8);

}

// Function to write to a register on LT8722 driver board
void write_reg(uint8_t reg, uint32_t value)
{
        typedef union
        {
          uint8_t   data_8[4];
          uint32_t  data_32;
          int32_t   data_32s;
        }dac_data;

      dac_data dac_data0;
           
      char tmp[8];
      dac_data0.data_32=value;
      
      w_data_8[0] =       reg;           //SPI Address
      w_data_8[1] = dac_data0.data_8[3]; //WD3
      w_data_8[2] = dac_data0.data_8[2]; //WD2
      w_data_8[3] = dac_data0.data_8[1]; //WD1
      w_data_8[4] = dac_data0.data_8[0]; //WD0

      transfer_8722(DW, w_data_8, r_data_8);
}

void byte_to_hex(byte value)
// convert a byte to two hex characters
{
  byte_to_hex_buffer[0]=hex_digits[value>>4];        // get upper nibble
  byte_to_hex_buffer[1]=hex_digits[(value & 0x0F)];  // get lower nibble
  byte_to_hex_buffer[2]='\0';                        // add NULL at end
}

byte read_hex()
// read 2 hex characters from the serial buffer and convert them to a byte
{
  byte data;
  hex_to_byte_buffer[2]=get_char();
  hex_to_byte_buffer[3]=get_char();
  data = strtol(hex_to_byte_buffer, NULL, 0);
  return(data);
}

char get_char()
// get the next character either from the serial port or the recording buffer
{
  char command='\0';
  if (recording_mode != playback)
  {
    // read a command from the serial port
    while (Serial.available() <= 0);
    return(Serial.read());
  }
  else
  {
    // read a command from the recording buffer
    if (recording_index < RECORDING_SIZE)
    {
      command = recording_buffer[recording_index++];
      // disregard loop commands during playback
      if (command == 'w') command='\1';
      if (command == 't') command='\1';
      if (command == 'v') command='\1';
      if (command == 'u') command='\1';
    }
    else
      command = '\0';
    if (command == '\0')
    {
      recording_index = 0;
      recording_mode = off;
    }
    return(command);
  }
}
int i = 0;
unsigned char pseudo_reset = 0;

///////////////Mode0 PID constants//////////////////
float set_resistance = 1080;    //Pre set the setpoint of thermo resistance
float kp_t = 1 ;   float ki_t = 0.005;   float kd_t = 0.000;
float PID_p_t = 0;    float PID_i_t = 0;    float PID_d_t = 0;
float uplim_t = 5;
float downlim_t = -5;
///////////////////////////////////////////////////

///////////////Mode1 PID constants//////////////////
float set_voltage = 0.74;      //Pre set the setpoint of phase signal
float kp_p = 8 ;   float ki_p = 0.01;   float kd_p = 0.;
float uplim_p = 6;
float downlim_p = -6;


float PID_p_p = 0;    float PID_i_p = 0;    float PID_d_p = 0;

///////////////////////////////////////////////////

void setup()
// Setup the program
{ 
  digitalWrite(QUIKEVAL_GPIO, HIGH);
  digitalWrite(QUIKEVAL_CS, HIGH);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  pinMode(QUIKEVAL_GPIO, OUTPUT);
  pinMode(QUIKEVAL_CS, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(QUIKEVAL_GPIO, HIGH);

  Serial.begin(115200);
  quikeval_SPI_init();
  quikeval_SPI_connect();   // Connect SPI to main data port

  quikeval_I2C_init();           // Configure the EEPROM I2C port for 100kHz SCK
  //Serial.print("hello\n");
  //Serial.flush();

  Time = millis();

  char command;
  command = 'U';
  switch (command)
  {
    case 'U':           //Entering the U command does a soft-start startup of the LT8722.
      //Serial.print("U \n");
      //reset dev 0
      write_reg(0x0,0x00004000);
      delayMicroseconds(100);
      write_reg(0x0,0x0000120D);
      delayMicroseconds(100);
      ///////////////////////////////////
      //Clear the SPIS_Status register
      write_reg(0x1,0x00000000);
      delayMicroseconds(100);
      write_reg(0x0,0x0000120D);
      delayMicroseconds(100);
      ///////////////////////////////////////
      // Ramp the LDR output from 0V to VDD/2
      ///////////////////////////////////////
      for(i_soft = 0xFFD00000; i_soft < 0xFFFF0000; i_soft = i_soft + 0x00010000)
      {
        write_reg(0x4,i_soft);
        delayMicroseconds(50);
       }
      // Finish off the ramp      
      write_reg(0x4,0xFFFF0000);
      delayMicroseconds(100);        
      write_reg(0x4,0x00000000);
      // LDR should now be at Vin/2
      ///////////////////////////////////
      //Clear the SPIS_Status register
      write_reg(0x1,0x00000000);
      delayMicroseconds(100);
      ///////////////////////////////////
      //Turn on the switcher
      write_reg(0x0,0x0000120F);     
      ///////////////////////////////////
      //Serial.print("The LT8722 has just been soft-started\n");          
      break;       
      /////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////
      //Power down command
      case 'u':
      Serial.print("u \n");
      ///////////////////////////////////
      //Turn off the switcher
      write_reg(0x0,0x0000120D);   
      ///////////////////////////////////
      //Turn off the LDR stage also
      write_reg(0x0,0x0000120C);     
      //Serial.print("The LT8722 has just had its buck stage and LDR stage powered off\n");   
      break; 
      default:
    // statements
       break;      
}
}

void loop()
{
  uint32_t iter=0;
  byte tx_data;
  byte rx_data;
  char command;
  int byte_count;
  long delay_count;
  byte w_ss_step = 0x01;
  char tmp[16];
  String inString = "";
  static float cumulativeResistance = 0;
  static uint8_t i=0;
  ////////////////// set the mode and temperature //////////////////////////////////
  if (Serial.available()>0){
    while(Serial.available()>0){
      serData = serData + (char)Serial.read();
      delay(1);                                       // It's better to delay 1ms between reading characters so as not to miss
    }
    if (serData[0]=='M'){                            //set the mode according to Serial input command, '0' is temperature mode and '1' is phase mode 
      String mode_set = serData.substring(1);
      mode = mode_set[0];
      }
    if (serData[0]=='t'){                            //set the temperature set point. Note that in the python code this command is, "sptemp"+str(temp_set_point), and substring[6] is exactly the "str(temp_set_point)"
      set_point = serData.substring(1);             
      float tset_point = set_point.toFloat();
      set_resistance = tset_point;
      }
    if (serData[0]=='p'){                           //set the phase set point
      set_point = serData.substring(1);
      float pset_point = set_point.toFloat();
      set_voltage = pset_point;
      }
    if (serData[0]=='f'){
      scan_freq_command = serData;
      freq_update = 1;
      }
    if (serData[0]=='m'){
      M0_switch = serData.substring(1).toInt();
      }

      
      serData = "";    
    }
    
  switch ( mode )
  { 
    //locking by temperature Mode
    case '0':  {                                          
     //////////////////////Ohmmeter PID Loop/////////////////////
     // First we read the real value of temperature
     // read the input on analog pin 1:
     while(i<AVERAGES)
     {
      uint16_t sensorValue = analogRead(A1);
      //Serial.println(sensorValue);
      // Convert the analog reading (which goes from 0 - 1023) to a resistance:
      float resistance_read = R_REF * sensorValue/(1024.0 - sensorValue);
      cumulativeResistance = cumulativeResistance + resistance_read;
      i++;
      //delay(100/AVERAGES);
      }
      float resistance = cumulativeResistance/AVERAGES;
      i=0;
      cumulativeResistance = 0;
      // print out the value you read:
      float T_real = 20+(resistance-1077.9)/4;
      //Serial.println(T_real);

      if (freq_update == 1){
        Serial.println(scan_freq_command);
        freq_update = 0;
      }     

    
      ///////////////////////Read phase signal////////////////////
     float PhaseValue = analogRead(A0);
     // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
     float voltage_read1 = PhaseValue * (5.0 / 1023.0);
     // print out the value you read:
     //Serial.println(voltage_read1);
     Serial.flush(); 
     delay(20);

      //Next we calculate the error between the setpoint and the real value
      PID_error_t = set_resistance - resistance;
      //Calculate the P value
      PID_p_t = kp_t * PID_error_t;
      //Calculate the I value in a range on +-5
      PID_i_t = PID_i_t + (ki_t * PID_error_t);
      if(5 < PID_i_t ){
        PID_i_t = 5;
        }
      else if( PID_i_t <-5 ){
        PID_i_t = -5;
        }
      //Serial.println(PID_i_t);
      
      //For derivative we need real time to calculate speed change rate
      timePrev = Time;                            // the previous time is stored before the actual time read
      Time = millis();                            // actual time read
      elapsedTime = (Time - timePrev) / 1000;
      //Now we can calculate the D calue
      PID_d_t = kd_t*((PID_error_t - previous_error)/elapsedTime);
      //Final total PID value is the sum of P + I + D
      PID_value = (PID_p_t + PID_i_t + PID_d_t);      //minus sign depends in the shape of error signal
      if(PID_value < downlim_t){ 
        PID_value = downlim_t;
        }
      if(PID_value > uplim_t){ 
        PID_value = uplim_t;
        }
      /////////////////////////////Apply control voltage/////////////////////////////////////////
        command = 'V';
        switch (command){   
          case 'V':
          iter=0;
          //Serial.print("V \n");
          //Serial.print("In DC routine \n");
          delay(1);
         //tempFloat = inString.toFloat();
         if (PID_value < 5 ){
          tempFloat = PID_value ;
          }
          output_voltage_dev(tempFloat/0.969);      //write voltage to the driver board
          //Serial.println(tempFloat);
          delayMicroseconds(100);  
          Serial.flush();  
          break; 
        }
        
       /////////////////////////////Serial printout/////////////////////////////////////////
       switch(M0_switch){
          case 0:     // muted temp stabilization
            break;

          case 1:     // monitored temp stabilization
            Serial.println(resistance);
            break;

          case 2:     // normal output 
            Serial.print(resistance);
            Serial.print(',');
            Serial.println(voltage_read1);

          default:
            break;
       }

        
      break;
    }


    ////////////////locking by phase Mode/////////////////////////////////
    case '1':  {                
        // read the input on analog pin 1:
        i=0;
        cumulativeResistance = 0; 
       while(i<AVERAGES)
       {
          uint16_t sensorValue = analogRead(A1);
          //Serial.println(sensorValue);
          // Convert the analog reading (which goes from 0 - 1023) to a resistance:
          float resistance_read = R_REF * sensorValue/(1024.0 - sensorValue);
          cumulativeResistance = cumulativeResistance + resistance_read;
          i++;
          //delay(100/AVERAGES);
       }
       
       float resistance = cumulativeResistance/AVERAGES;
                                      
       // Read the real value of Phase
       // read the input on analog pin 0:
       float sensorValue = analogRead(A0);
       // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
       float voltage_read2 = sensorValue * (5.0 / 1023.0);

       Serial.flush(); 
       delay(20);

       //Next we calculate the error between the setpoint and the real value
       PID_error_p = set_voltage - voltage_read2;
       //Calculate the P value
       PID_p_p = kp_p * PID_error_p;
       //Calculate the I value in a range on +-5
       PID_i_p = PID_i_p + (ki_p * PID_error_p);
       if(5 < PID_i_p )
       {
        PID_i_p = 5;
        }
       if( PID_i_p <-5 )
       {
        PID_i_p = -5;
        }
        //Serial.println(PID_i_p);
        //For derivative we need real time to calculate speed change rate
        timePrev = Time;                            // the previous time is stored before the actual time read
        Time = millis();                            // actual time read
        elapsedTime = (Time - timePrev) / 1000; 
        //Now we can calculate the D value
        PID_d_p = kd_p*((PID_error_p - previous_error)/elapsedTime);
        //Final total PID value is the sum of P + I + D
        PID_value = -(PID_p_p + PID_i_p + PID_d_p);      //minus sign depends in the shape of error signal
        if(PID_value < downlim_p)
        { 
          PID_value = downlim_p;
          }
        if(PID_value > uplim_p)
        { 
          PID_value = uplim_p;
          }
          
       ///////////////////////////Apply control voltage/////////////////////////////////////////
          command = 'V';
        switch (command){   
          case 'V':
          iter=0;
          //Serial.print("V \n");
          //Serial.print("In DC routine \n");
          delay(1);
         //tempFloat = inString.toFloat();
         if (PID_value < 5 ){
          tempFloat = PID_value ;
          }
          output_voltage_dev(tempFloat/0.969);
          
          delayMicroseconds(100);  
          Serial.flush();  
          break; 
          }
          //////////////////////////Serial printout///////////////////////////////////////////
          Serial.print(voltage_read2);
          Serial.print(',');
          Serial.print(tempFloat/0.969);
          Serial.print(',');
          Serial.println(resistance);
          
       break;
    }
    
    default:
     {   
     break;
    }
  }
}
