/**************************************************************************************/
/*
  TwoWire.h - master TWI/I²C library for ESP8266 Arduino

  Modified          2012 by Todd Krein (todd@krein.org) to implement repeated starts
  Modified December 2014 by Ivan Grokhotkov (ivan@esp8266.com) - esp8266 support
  Modified April    2015 by Hrsto Gochkov (ficeto@ficeto.com) - alternative esp8266 support
  Modified          2019 by enjoyneering79, source code: https://github.com/enjoyneering/

  Specials pins are required:
  Board:                                     SDA        SCL        Level
  ESP8266................................... GPIO4      GPIO5      3.3v/5v
  ESP8266 ESP-01............................ GPIO0/D5   GPIO2/D3   3.3v/5v
  NodeMCU 1.0, WeMos D1 Mini................ GPIO4/D2   GPIO5/D1   3.3v/5v

  NOTE:
  - I2C bus drivers are "open drain", meaning that they can pull the
    corresponding signal line low, but cannot drive it high. Thus, there can
    be no bus contention where one device is trying to drive the line high
    while another tries to pull it low, eliminating the potential for damage
    to the drivers or excessive power dissipation in the system. Each signal
    line has a pull-up resistor on it, to restore the signal to high when no
    device is asserting it low.

  Copyright (c) 2006 Nicholas Zambetti. All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/**************************************************************************************/

#ifndef TwoWire_h
#define TwoWire_h

#include <inttypes.h>
#include "Stream.h"

/* 
The arduino toolchain includes library headers before it includes your sketch.
Unfortunately, you cannot #define in a sketch & get it in the library.
*/
//defined TWI_I2C_DISABLE_INTERRUPTS //uncomment to disable interrupts during read/write

#ifndef TWI_I2C_BUFFER_LENGTH
#define TWI_I2C_BUFFER_LENGTH 32     //32-bytes rx buffer + 32-bytes tx buffer, 64-bytes total
#endif


class TwoWire : public Stream
{
  public:
    TwoWire();

             void   pins(int sda, int scl) __attribute__((deprecated)); //use "begin(sda, scl)" in new projects
             void   begin(uint8_t sda, uint8_t scl);
             void   begin(void);
             void   setClock(uint32_t frequency);
             void   setClockStretchLimit(uint32_t limit);

             void   beginTransmission(uint8_t address);
             void   beginTransmission(int address);

    virtual size_t  write(uint8_t data);
    virtual size_t  write(const uint8_t *buffer, size_t quantity);

  //inline  size_t  write(int data)           {return write((uint8_t)data);}
  //inline  size_t  write(unsigned int data)  {return write((uint8_t)data);}
  //inline  size_t  write(long data)          {return write((uint8_t)data);}
  //inline  size_t  write(unsigned long data) {return write((uint8_t)data);}

            using   Print::write;

            uint8_t endTransmission(bool sendStop);
            uint8_t endTransmission(void);

            uint8_t requestFrom(uint8_t address, uint8_t quantity, bool    sendStop);
            uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop);
            uint8_t requestFrom(uint8_t address, uint8_t quantity);
            uint8_t requestFrom(int     address, int     quantity, bool    sendStop);
            uint8_t requestFrom(int     address, int     quantity, int     sendStop);
            uint8_t requestFrom(int     address, int     quantity);
    virtual int     available(void);
    virtual int     read(void);

    virtual int     peek(void);
            void    flushRX(void);
            void    flushTX(void);
    virtual void    flush(void);
            uint8_t status(void);

            void    onReceive(void (*)(int));
            void    onRequest(void (*)(void));

  private:
    static uint8_t _rxBuffer[];
    static uint8_t _rxBufferIndex;
    static uint8_t _rxBufferLength; 

    static uint8_t _txBuffer[];
    static uint8_t _txAddress;
    static uint8_t _txBufferIndex;
    static uint8_t _txBufferLength;
    static bool    _transmitting;

    static void onReceiveService(uint8_t *inBytes, int numBytes);
    static void onRequestService(void);
    static void (*user_onRequest)(void);
    static void (*user_onReceive)(int);
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_TWOWIRE)
extern TwoWire Wire;
#endif

#endif
