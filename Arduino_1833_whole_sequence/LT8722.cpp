/*!
  General BMS Library
@verbatim

@endverbatim

Copyright 2018(c) Analog Devices, Inc.

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

Copyright 2017 Linear Technology Corp. (LTC)
***********************************************************/
#include <stdint.h>
#include "LT8722.h"
#include "lt8722hardware.h"
#include <Arduino.h>
#include "Linduino.h"



/********************************************************************************
                    PEC CALCULATION - LOOK UP TABLE 
This function takes ~8.75us to calculate PEC for 6 bytes input data.
*********************************************************************************/
uint8_t pec_8_calc(unsigned char * data, uint8_t byte_num)
{
	uint8_t PEC = 0;
	for (uint8_t i = 0; i < byte_num; i++)
	{
		PEC = CRC_8_TABLE[(PEC ^ data[i])];
	}
	return PEC;
}

void transfer_8722(uint8_t command, uint8_t * tx_data, uint8_t * rx_data)
{	
	uint8_t command_len = 0;
	uint8_t *cmd;
	uint8_t cmd_pec;	
	
	switch(command)
	{
	  case SQ:
		command_len = 4;
        cmd = (uint8_t *)malloc(command_len*sizeof(uint8_t));		
		cmd[0] = 0xF0 | (command << 1);
		cmd[1] = (tx_data[0] << 1) & 0xFE;
		cmd[2] = pec_8_calc(cmd, 2);
		cmd[3] = 0;
		break;
	  case DW:
		command_len = 8;
        cmd = (uint8_t *)malloc(command_len*sizeof(uint8_t));		
		cmd[0] = 0xF0 | (command << 1);
		cmd[1] = (tx_data[0] << 1) & 0xFE;
		cmd[2] = tx_data[1];
		cmd[3] = tx_data[2];
		cmd[4] = tx_data[3];
		cmd[5] = tx_data[4];
		cmd[6] = pec_8_calc(cmd, 6);
		cmd[7] = 0;
		break;
	  case DR:
		command_len = 8;
        cmd = (uint8_t *)malloc(command_len*sizeof(uint8_t));		
		cmd[0] = 0xF0 | (command << 1);
		cmd[1] = (tx_data[0] << 1) & 0xFE;
		cmd[2] = pec_8_calc(cmd, 2);
		cmd[3] = 0;	
		cmd[4] = 0;	
		cmd[5] = 0;	
		cmd[6] = 0;	
		cmd[7] = 0;			
		break;
	  default:
		Serial.print("Incorrect Command\r\n");
		break;
		
	}
	//transfer data
	//if(!command_len)
	//{
/*
    char tmp[16];
		Serial.print("MOSI:");
		for(uint8_t i = 0; i < command_len; i++)
		{ 
      sprintf(tmp, "%02X",cmd[i]);
      Serial.print(tmp);
			//Serial.print(cmd[i], HEX);
			//Serial.print(" ");
		}
		Serial.print("\r\n");
*/
   
		cs_low(CS_PIN);
		spi_transfer(cmd, rx_data, command_len);
		cs_high(CS_PIN);
	//}	
	free(cmd);
}
