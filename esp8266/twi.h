/**************************************************************************************/
/*
  twi.h - Software I2C library for esp8266

  Modified October 2017 by enjoyneering79, sourse code: https://github.com/enjoyneering/

  This library is software bitbang emulation of Master I2C bus protocol, specials pins
  are required to interface.

  Connect slave to pins:    SDA        SCL
  ESP8266 ESP-01:           GPIO0/D5   GPIO2/D3
  NodeMCU 1.0:              GPIO4/D2   GPIO5/D1
  WeMos D1 Mini:            GPIO4/D2   GPIO5/D1

  NOTE:
  - I2C bus drivers are “open drain”, meaning that they can pull the
    corresponding signal line low, but cannot drive it high. Thus, there can
    be no bus contention where one device is trying to drive the line high
    while another tries to pull it low, eliminating the potential for damage
    to the drivers or excessive power dissipation in the system. Each signal
    line has a pull-up resistor on it, to restore the signal to high when no
    device is asserting it low.

  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.
 
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

#ifndef SI2C_h
#define SI2C_h
#include "Arduino.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define I2C_SDA_POLLING_LIMIT       20    //qnt of tries to release I2C bus if slave locked SDA low

#define I2C_NACK                    HIGH  //1
#define I2C_ACK                     LOW   //0

#define I2C_OK                      0     //bus OK
#define I2C_SDA_HELD_LOW            1     //SDA held low by another device, no procedure available to recover
#define I2C_SDA_HELD_LOW_AFTER_INIT 2     //SDA held low beyond slave clock stretch time
#define I2C_SCL_HELD_LOW            3     //SCL held low by another device, no procedure available to recover
#define I2C_SCL_HELD_LOW_AFTER_READ 4     //SCL held low beyond slave clock stretch time

void    twi_init(uint8_t sda, uint8_t scl);
void    twi_setClock(uint32_t freq);
void    twi_setClockStretchLimit(uint32_t limit);
uint8_t twi_writeTo(uint8_t address, uint8_t *buffer, uint8_t length, bool sendStop);
uint8_t twi_readFrom(uint8_t address, uint8_t *buffer, uint8_t length, bool sendStop);
uint8_t twi_status();

#ifdef __cplusplus
}
#endif

#endif
