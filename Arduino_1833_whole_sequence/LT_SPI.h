/*!
LT_SPI: Routines to communicate with ATMEGA328P's hardware SPI port.

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

/*! @file
    @ingroup LT_SPI
    Library Header File for LT_SPI: Routines to communicate with ATMEGA328P's hardware SPI port.
*/

#ifndef LT_SPI_H
#define LT_SPI_H

#include <stdint.h>

//! @name SPI CLOCK DIVIDER CONSTANTS
//! @{
#define SPI_CLOCK_DIV4    0x00  // 4 Mhz
#define SPI_CLOCK_DIV16   0x01  // 1 Mhz
#define SPI_CLOCK_DIV64   0x02  // 250 khz
#define SPI_CLOCK_DIV128  0x03  // 125 khz
#define SPI_CLOCK_DIV2    0x04  // 8 Mhz
#define SPI_CLOCK_DIV8    0x05  // 2 Mhz
#define SPI_CLOCK_DIV32   0x06  // 500 khz
//! @}

//! @name SPI HARDWARE MODE CONSTANTS
//! @{
#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C
//! @}

//! @name SPI SET MASKS
//! @{
#define SPI_MODE_MASK      0x0C    // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK     0x03    // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK   0x01    // SPI2X = bit 0 on SPSR
//! @}

// Connect SPI pins to QuikEval connector through the Linduino MUX. This will disconnect I2C.
void quikeval_SPI_connect();

// Initializes SPI
void quikeval_SPI_init();

// Setup the processor for hardware SPI communication.
// Must be called before using the other SPI routines.
void spi_enable(uint8_t spi_clock_divider);

// Disable the SPI hardware port
void spi_disable();

// Write a data byte using the SPI hardware
void spi_write(int8_t data);

// Read and write a data byte using the SPI hardware
int8_t spi_read(int8_t data);

#endif  // LT_SPI_H
