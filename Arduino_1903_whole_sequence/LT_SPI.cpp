/*!
LT_SPI: Routines to communicate with ATMEGA328P's hardware SPI port.

@verbatim

LT_SPI implements the low level master SPI bus routines using
the hardware SPI port.

SPI Frequency=(CPU Clock frequency)/(16+2(TWBR)*Prescaler)
SPCR = SPI Control Register (SPIE SPE DORD MSTR CPOL CPHA SPR1 SPR0)
SPSR = SPI Status Register (SPIF WCOL - - - - - SPI2X)

Data Modes:
CPOL  CPHA  Leading Edge    Trailing Edge
0      0    sample rising   setup falling
0      1    setup rising    sample falling
1      0    sample falling  setup rising
1      1    sample rising   setup rising

CPU Frequency = 16Mhz on Arduino Uno
SCK Frequency
SPI2X  SPR1  SPR0  Frequency  Uno_Frequency
  0      0     0     fosc/4     4 Mhz
  0      0     1     fosc/16    1 Mhz
  0      1     0     fosc/64    250 khz
  0      1     1     fosc/128   125 khz
  0      0     0     fosc/2     8 Mhz
  0      0     1     fosc/8     2 Mhz
  0      1     0     fosc/32    500 khz

@endverbatim

REVISION HISTORY
$Revision: 1479 $
$Date: 2013-04-11 15:38:46 -0700 (Thu, 11 Apr 2013) $

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment 
to the open-source community.  Please, visit http://www.arduino.cc and 
http://store.arduino.cc , and consider a purchase that will help fund their 
ongoing work.
*/

//! @defgroup LT_SPI LT_SPI: Routines to communicate with ATMEGA328P's hardware SPI port.

/*! @file
    @ingroup LT_SPI
    Library for LT_SPI: Routines to communicate with ATMEGA328P's hardware SPI port.
*/

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"

//! Connect SPI pins to QuikEval connector through the Linduino MUX. This will disconnect I2C.
void quikeval_SPI_connect()  
{
  output_high(QUIKEVAL_CS);                        // Pull Chip Select High

  // Enable Main SPI
  pinMode(QUIKEVAL_MUX_MODE_PIN, OUTPUT);
  digitalWrite(QUIKEVAL_MUX_MODE_PIN, LOW);
}

//! Configure the SPI port for 4Mhz SCK.
//! This function or spi_enable() must be called 
//! before using the other SPI routines.
void quikeval_SPI_init(void)  // Initializes SPI
{
  spi_enable(SPI_CLOCK_DIV4);           // Configure the spi port for 4Mhz SCK
}

//! Setup the processor for hardware SPI communication.
//! Must be called before using the other SPI routines.
//! Alternatively, call quikeval_SPI_connect(), which automatically 
//! calls this function.
void spi_enable(uint8_t spi_clock_divider) //!< Configures SCK frequency. Use constant defined in header file.               
{
  pinMode(SCK, OUTPUT);         // Setup SCK as output
  pinMode(MOSI, OUTPUT);        // Setup MOSI as output
  pinMode(QUIKEVAL_CS, OUTPUT);          // Setup CS as output
  output_low(SCK);
  output_low(MOSI);
  output_high(QUIKEVAL_CS);
  SPCR |= _BV(MSTR);                    // Set the SPI port to master mode
  // Set the SPI hardware rate
  SPCR = (SPCR & ~SPI_CLOCK_MASK) | (spi_clock_divider & SPI_CLOCK_MASK);
  SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((spi_clock_divider >> 2) & SPI_2XCLOCK_MASK);
  SPCR |= _BV(SPE);                     // Enable the SPI port
}

//! Disable the SPI hardware port
void spi_disable()
{
  SPCR &= ~_BV(SPE);
}

//! Write a data byte using the SPI hardware
void spi_write(int8_t  data)  //!< Byte to be written to SPI port
                
{
  SPDR = data;                  // Start the SPI transfer
  while (!(SPSR & _BV(SPIF)));  // Wait until transfer complete
}

//! Read and write a data byte using the SPI hardware
int8_t spi_read(int8_t  data) //!< The data byte to be written
//! @return the data byte read
{
  SPDR = data;                  // Start the SPI transfer
  while (!(SPSR & _BV(SPIF)));  // Wait until transfer complete
  return SPDR;                  // Return the data read
}
