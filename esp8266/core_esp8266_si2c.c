/**************************************************************************************/
/*
  si2c.c - Software I2C library for esp8266

  Modified October 2017 by enjoyneering79, sourse code: https://github.com/enjoyneering/

  This library is software/bit-bang emulation of Master I2C bus protocol, specials pins
  are required to interface. Connect slave to pins:

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

#include "twi.h"
#include "pins_arduino.h"
#include "wiring_private.h"

#ifndef FCPU80
#define FCPU80 80000000UL
#endif

#if F_CPU == FCPU80
#define TWI_CLOCK_STRETCH_MULTIPLIER 3
#else
#define TWI_CLOCK_STRETCH_MULTIPLIER 6
#endif

#define SDA_LOW()  (GPES = (1 << twi_sda))        //enable  SDA (becomes output & since GPO is 0 for the pin, it will pull the line low)
#define SDA_HIGH() (GPEC = (1 << twi_sda))        //disable SDA (becomes input  & since it has pullup it will go high)
#define SDA_READ() ((GPI & (1 << twi_sda)) != 0)
#define SCL_LOW()  (GPES = (1 << twi_scl))
#define SCL_HIGH() (GPEC = (1 << twi_scl))
#define SCL_READ() ((GPI & (1 << twi_scl)) != 0)

static   uint8_t  twi_sda               = 0;      //ESP8266 ESP-01: GPIO0/D5, NodeMCU 1.0 & WeMos D1 Mini: GPIO4/D2
static   uint8_t  twi_scl               = 0;      //ESP8266 ESP-01: GPIO2/D3, NodeMCU 1.0 & WeMos D1 Mini: GPIO5/D1
static   uint32_t twi_clockStretchLimit = 0;
         uint8_t  twi_dcount            = 0;
         uint32_t preferred_si2c_clock  = 100000; //default i2s speed 100kHz

static   bool     collision             = false;  //shows if bit was successfuly read from slave

/**************************************************************************/
/*
    twi_setClock()

    Sets i2c clock speed
*/
/**************************************************************************/
void twi_setClock(uint32_t freq)
{
  preferred_si2c_clock = freq;

  #if F_CPU == FCPU80
  if      (freq <= 50000)  twi_dcount = 38; //~50KHz
  else if (freq <= 100000) twi_dcount = 19; //~100KHz
  else if (freq <= 200000) twi_dcount = 8;  //~200KHz
  else if (freq <= 300000) twi_dcount = 3;  //~300KHz
  else if (freq <= 400000) twi_dcount = 1;  //~400KHz
  else                     twi_dcount = 1;  //~400KHz
  #else
  if      (freq <= 50000)  twi_dcount = 64; //~50KHz
  else if (freq <= 100000) twi_dcount = 32; //~100KHz
  else if (freq <= 200000) twi_dcount = 14; //~200KHz
  else if (freq <= 300000) twi_dcount = 8;  //~300KHz
  else if (freq <= 400000) twi_dcount = 5;  //~400KHz
  else if (freq <= 500000) twi_dcount = 3;  //~500KHz
  else if (freq <= 600000) twi_dcount = 2;  //~600KHz
  else                     twi_dcount = 1;  //~700KHz
  #endif
}

/**************************************************************************/
/*
    twi_setClockStretchLimit

    Sets SCL max stretch time
*/
/**************************************************************************/
void twi_setClockStretchLimit(uint32_t limit)
{
  twi_clockStretchLimit = limit * TWI_CLOCK_STRETCH_MULTIPLIER;
}

/**************************************************************************/
/*
    twi_delay, in usec
*/
/**************************************************************************/
static void twi_delay(uint8_t value)
{
  uint16_t reg = 0;

  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"

  for (uint8_t i = 0; i < value; i++) reg = GPI;

  #pragma GCC diagnostic pop
}

/**************************************************************************/
/*
    clockStretch()

    Checks for SCL locked low by slave

    NOTE:
    - when slave can't keep-up with master it locks SCL line low, it's
      a sign that slave needs more time.
*/
/**************************************************************************/
static bool clockStretch(void)
{
  int32_t pollCounter = twi_clockStretchLimit;

  /* SCL is low, slave needs more time */
  while (SCL_READ() == LOW)
  {
    if (pollCounter-- < 0) return false; //error handler, ERROR - SCL locked!!!
  }
  return true;                           //SCL released!!!
}

/**************************************************************************/
/*
    freeBus()

    Dirty hack to release I2C bus if slave locked SDA low

    NOTE:
    - if for some reason previouse reading from diff. slave is not done yet
    - the first SCL should be 1 / (tLOW + tHIGH) = bus frequency, so
      1 / (4.700 + 4.000) = 115kHz & real 1 / (4.875 + 4.125) = 111.111kHz
    - see NXP UM10204 p.48 and p.50 for timing details
*/
/**************************************************************************/
static bool freeBus(void)
{
  int8_t pollCounter = I2C_SDA_POLLING_LIMIT;

  /* SDA is low, I2C bus is locked */
  while (SDA_READ() == LOW)
  {
    if (pollCounter-- < 0) return false;      //error handler, ERROR - SDA busy!!!

    SCL_LOW();                                //becomes output & pulls the line low
    twi_delay(twi_dcount - 6);                //tLOW >= 4.7usec, LOW period of the SCL (real time 4.875usec)

    SCL_HIGH();                               //release the SCL bus, so we can read a bit or ack/nack answer from slave
    if (clockStretch() != true) return false; //stretching the clock slave is buuuuuusy, ERROR - SCL busy!!!
    twi_delay(twi_dcount - 5);                //tHIGH = 4.0usec, HIGH period of the SCL (real time - 4.125usec)
  }
  return true;                                //SDA released!!!
}

/**************************************************************************/
/*
    twi_init

    Initialisation of soft/bitbang master I2C bus protocol

    NOTE:
    - I2C bus drivers are “open drain”, meaning that they can pull the
      corresponding signal line low, but cannot drive it high. Thus, there can
      be no bus contention where one device is trying to drive the line high
      while another tries to pull it low, eliminating the potential for damage
      to the drivers or excessive power dissipation in the system. Each signal
      line has a pull-up resistor on it, to restore the signal to high when no
      device is asserting it low.
*/
/**************************************************************************/
void twi_init(uint8_t sda, uint8_t scl)
{
  twi_sda = sda;
  twi_scl = scl;

  SCL_HIGH();                         //becomes input has pullup & pulls the line high
  SDA_HIGH();                         //becomes input has pullup & pulls the line high
  delay(20);                          //some slave needs >= 15msec to reach the idle state

  twi_setClock(preferred_si2c_clock); //~100KHz, by default
  twi_setClockStretchLimit(230);      //stretch SCL limit, in usec

  freeBus();                          //dirty hack to release locked SDA line
}

/**************************************************************************/
/*
    twi_write_start()
    
    Sends start pulse & returns false if SCL or SDA is busy

    NOTE:
    - START conditions are always generated by the master
    - the bus is considered to be busy after the START condition.
    - see TI SLVA704 fig.5 on p.4 for more details
    - see NXP UM10204 p.48 and p.50 for timing details
*/
/**************************************************************************/
static bool twi_write_start(void)
{
  /* start SCL & SDA routine */
  SCL_HIGH();                            //becomes input has pullup & pulls the line high, SCL FIRST DO NOT TOUCH
  SDA_HIGH();                            //becomes input has pullup & pulls the line high
  twi_delay(twi_dcount - 14);            //tSU;STA & tBUF >= 4.7usec, bus free time between STOP & START condition (real time - 5.25usec)

  /* check if SDA line is blocked */
  if (freeBus() != true) return ~I2C_OK; //dirty hack to release locked SDA line

  /* continue SCL & SDA routine */
  SDA_LOW();                             //becomes output & pulls the line low
  twi_delay(twi_dcount - 1);             //tHD;STA >= 4.0usec, START hold time (real time - 4.375usec)
  SCL_LOW();

  return I2C_OK;                         //success!!!
}

/**************************************************************************/
/*
    twi_write_stop()

    Sends stop pulse, returns != I2C_OK if SCL busy

    NOTE:
    - STOP conditions are always generated by the master
    - bus is free only after the STOP condition
    - see TI SLVA704 fig.5 on p.4 for more details
    - see NXP UM10204 p.48 and p.50 for timing details
*/
/**************************************************************************/
static uint8_t twi_write_stop(void)
{
  /* start SCL & SDA routine */
  SCL_LOW();                                  //becomes output & pulls the line low, SCL FIRST DO NOT TOUCH
  SDA_LOW();                                  //becomes output & pulls the line low
  twi_delay(twi_dcount - 3);                  //tLOW >= 4.7usec, LOW period of the SCL (real time - 5.125usec)

  /* continue SCL routine */
  SCL_HIGH();                                 //release the SCL line, becomes input has pullup & pulls the line high
  if (clockStretch() != true) return ~I2C_OK; //stretching the clock slave is buuuuuusy, ERROR - SCL busy!!!
  twi_delay(twi_dcount - 4);                  //tSU;STO >= 4.0usec, set-up time for STOP condition (real time - 4.375usec)

  /*continue SDA routine */
  SDA_HIGH();                                 //finish the STOP by releasing SDA line

  return I2C_OK;                              //success!!!
}

/**************************************************************************/
/*
    twi_write_bit()

    Writes a bit, returns != I2C_OK if SCL busy

    NOTE:
    - data is stable and valid for read when SCL line is HIGH
    - the HIGH or LOW state of the SDA line can only change when the SCL
      line is LOW, one clock pulse is generated for each data bit transferred
    - if a slave cannot receive or transmit another complete byte of data it
      can hold the clock line SCL LOW to force the master into a wait state,
      data transfer continues when the slave is ready for another byte of
      data and releases clock line SCL
    - the first SCL should be 1 / (tLOW + tHIGH) = bus frequency, so
      1 / (4.700 + 4.000) = 115kHz & real 1 / (4.875 + 4.125) = 111.111kHz
    - see TI SLVA704, fig.6 on p.5
    - see NXP UM10204 p.48 and p.50 for timing details
*/
/**************************************************************************/
static bool twi_write_bit(bool txBit)
{
  /* start SCL routine */
  SCL_LOW();
  twi_delay(twi_dcount - 5);                         //tLOW >= 4.7usec, LOW period of the SCL (real time - 5.125usec)

  /* start SDA routine, set the bit */
  switch (txBit)
  {
    case HIGH:
      SDA_HIGH();                                    //becomes input has pullup & pulls the line high
      break;

    case LOW:
      SDA_LOW();                                     //becomes output & pulls the line low
      break;
  }
  twi_delay(1);                                      //tSU;DAT >= 250nsec, data set-up time

  /* continue SCL routine, bit is valid for read */
  SCL_HIGH();                                        //try to release the SCL line, becomes input has pullup & pulls the line high
  if (clockStretch() == false) return ~I2C_OK;       //stretching the clock slave is buuuuuusy, ERROR - SCL busy!!!
  twi_delay(twi_dcount - 4);                         //tHIGH >= 4.0usec, HIGH period of the SCL (real time - 4.375usec)
  SCL_LOW();
 
  return I2C_OK;                                     //success!!!
}

/**************************************************************************/
/*
    twi_read_bit()

    Reads a bit or NACK/ACK(1/0) if SCL busy

    NOTE:
    - data is stable and valid for read when SCL line is HIGH
    - the HIGH or LOW state of the SDA line can only change when the SCL
      line is LOW, one clock pulse is generated for each data bit transferred
    - if a slave cannot receive or transmit another complete byte of data it
      can hold the clock line SCL LOW to force the master into a wait state,
      data transfer continues when the slave is ready for another byte of
      data and releases clock line SCL
    - when SDA remains HIGH during this 9-th clock pulse, this is defined
      as the Not Acknowledge signal. The master can then generate either a
      STOP condition to abort the transfer, or a repeated START condition
      to start a new transfer. There are five conditions that lead to the
      generation of a NACK
    - the first SCL should be 1 / (tLOW + tHIGH) = bus frequency, so
      1 / (4.700 + 4.000) = 115kHz & real 1 / (4.875 + 4.125) = 111.111kHz
    - see TI SLVA704, fig.6 on p.5 or NXP UM10204, fig.4 on p.9 for more details
    - see NXP UM10204 p.48 and p.50 for timing details
*/
/**************************************************************************/
static uint8_t twi_read_bit(void)
{
  bool rxBit = 0;

  /* start SCL & SDA routine */
  SCL_LOW();                                            //becomes output & pulls the line low
  twi_delay(twi_dcount - 3);                            //tLOW >= 4.7usec, LOW period of the SCL (real time - 5.000usec)

  /* continue SCL routine */
  SDA_HIGH();                                           //becomes input & release the SDA line, so slave can send bit or ack/nack answer
  SCL_HIGH();                                           //release the SCL bus, so we can read a bit or ack/nack answer from slave
  if (clockStretch() == false) return I2C_SCL_HELD_LOW; //returns 3

  /* continue SDA routine, read the data */
  rxBit = SDA_READ();                                   //read 7..0 data bit or 8-th NACK/ACK = 1/0 bit, 9 bits in total
  twi_delay(twi_dcount - 6);                            //tHIGH >= 4.0usec, HIGH period of the SCL (real time - 4.375usec)
  switch (rxBit)                                        //during "twi_write_byte()" removes spike after reading 9-th NACK/ACK bit 
  {
    case HIGH:
      SDA_HIGH();     
      break;

    case LOW:
      SDA_LOW();       
      break;
  }
  SCL_LOW();

  return rxBit;                                         //return bit!!!
}

/**************************************************************************/
/*
    twi_write_byte()

    Write a byte, returns ACK/NACK

    NOTE:
    - data is transferred with the most significant bit (MSB) first
    - when SDA remains HIGH during this 9-th clock pulse, this is defined
      as NACK (Not Acknowledge). The master can then generate either a
      STOP condition to abort the transfer, or a repeated START condition
      to start a new transfer.
    - see NXP UM10204 p.48 and p.50 for timing details
*/
/**************************************************************************/
static bool twi_write_byte(uint8_t txByte)
{
  /* write the byte, in order MSB->LSB (7..0 bit) */
  for (int8_t i = 7; i >= 0; i--)
  {
    if (twi_write_bit(bitRead(txByte, i)) != I2C_OK) return I2C_NACK; //write the byte in order MSB->LSB (7..0 bit), ERROR - NACK!!!
  }

  if (twi_read_bit() == I2C_ACK) return I2C_ACK;                      //reads 9-th bit NACK/ACK

  return I2C_NACK;
}

/**************************************************************************/
/*
    twi_read_byte()

    Reads a byte & sends NACK/ACK after

    NOTE:
    - data is transferred with the most significant bit (MSB) first
    - when SDA remains HIGH during this 9-th clock pulse, this is defined
      as NACK (Not Acknowledge). The master can then generate either a
      STOP condition to abort the transfer, or a repeated START condition
      to start a new transfer.
    - 1 is NACK
    - 0 is ACK
*/
/**************************************************************************/
static uint8_t twi_read_byte(bool ack_nack)
{
  uint8_t rxByte = 0;
  uint8_t rxBit  = 0;

  /* read the byte, in order MSB->LSB (7..0 bit) */
  for (uint8_t i = 0; i < 8; i++)
  {
    rxBit = twi_read_bit();

    switch (rxBit)
    {
      case HIGH:
        rxByte = (rxByte << 1) | rxBit;
        break;

      case LOW:
        rxByte = rxByte << 1;
        break;

      case I2C_SCL_HELD_LOW:
        collision = true;                //SCL busy!!!
        return 0;
        break;
    }
  }

  /* write 9-th NACK/ACK bit */
  if (twi_write_bit(ack_nack) != I2C_OK)
  {
    collision = true;                    //SCL busy!!!
    return 0;
  }

  collision = false;
  return rxByte;                         //return byte!!!
}

/**************************************************************************/
/*
    twi_writeTo()

    Fills in txBuffer & returns code

    NOTE:
    - byte is transferred with the most significant bit (MSB) first
    - when master sets SDA HIGH during this 9-th clock pulse, this is
      defined as NACK (Not Acknowledge). The master generate STOP condition
      to abort the transfer
    - if master sets SDA LOW during this 9-th clock pulse, this is defined
      as ACK (Acknowledge). The master generate REPEATE START condition to
      start a new transfer.
    - regardless of the number of start conditions during one transfer
      the transfer must be ended by exactly one stop condition followed
      by NACK.
    - default buffer 16 bytes, see Wire.h

    Returned codes:
     0 - success
     1 - data too long to fit in transmit data16
     2 - received NACK on transmit of address
     3 - received NACK on transmit of data
     4 - line is busy
*/
/**************************************************************************/
uint8_t twi_writeTo(uint8_t address, uint8_t *buffer, uint8_t length, bool sendStop)
{
  //noInterrupts();                            //preventing interrupts

  /* send start */
  if (twi_write_start() != I2C_OK) return 4;   //line is busy!!!


  /* write address */
  if (twi_write_byte(address << 1) != I2C_ACK) //write address = (address << 1 + 0)
  {
    twi_write_stop();                          //always use stop after "twi_write_start()"
    return 2;                                  //received NACK during address transmission!!!
  }

  /* write n bytes */
  for(uint8_t i = 0; i < length; i++)
  {
    if (twi_write_byte(buffer[i]) != I2C_ACK)  //error handler
    {
      twi_write_stop();                        //always use stop after "twi_write_start()"
      return 3;                                //received NACK during data transmission!!!
    }
  }

  /* send stop */
  if (sendStop == true) twi_write_stop();

  //interrupts();

  return 0;                                    //success!!!
}

/**************************************************************************/
/*
    twi_readFrom()

    Fills in rxBuffer & returns qnt of received bytes or 0 if bus error

    NOTE:
    - default buffer 16 bytes, see Wire.h
*/
/**************************************************************************/
uint8_t twi_readFrom(uint8_t address, uint8_t *buffer, uint8_t length, bool sendStop)
{
  //noInterrupts();                                     //preventing interrupts

  if (length <= 0) length = 1;                          //safety check

  /* send start */
  if (twi_write_start() != I2C_OK) return 0;            //line is busy!!!


  /* write address */
  if (twi_write_byte((address << 1) | 0x01) != I2C_ACK) //read address = (address << 1 + 1)
  {
    twi_write_stop();                                   //always use stop after "twi_write_start()"
    return 0;                                           //received NACK during address transmition!!!
  }

  /* reads n-bytes from slave, except last one */
  for (uint8_t i = 0; i < (length - 1); i++)
  {
    buffer[i] = twi_read_byte(I2C_ACK);                 //sends ACK, asuming byte successfully received, slave can sent another one
    if (collision == true)                              //error reading n-byte from slave
    {
      twi_write_stop();                                 //always use stop after "twi_write_start()"
      return i;                                         //return qnt of successfully reserved of bytes
    }
  }

  /* reads last byte */
  switch (sendStop)                                     //"repeated START" conditions
  {
    case HIGH:
      buffer[(length - 1)] = twi_read_byte(I2C_NACK);   //always sends NACK before stop
      twi_write_stop();
      break;

    case LOW:
      buffer[(length - 1)] = twi_read_byte(I2C_ACK);    //sends ACK, for "repeated START"
      break;
  }

  if (collision == true)                                //error reading "last byte" from slave
  {
    if (sendStop == LOW) twi_write_stop();              //always use stop after "twi_write_start()"
    return 0;
  }
  //interrupts();
  return length;                                        //return qnt successfully reserved of bytes
}

/**************************************************************************/
/*
    twi_status()

    returned code:
     - I2C_OK                      0, OK
     - I2C_SDA_HELD_LOW            1, SDA held low by another device, no procedure available to recover
     - I2C_SDA_HELD_LOW_AFTER_INIT 2, SDA held low beyond slave clock stretch time
     - I2C_SCL_HELD_LOW            3, SCL held low by another device, no procedure available to recover
     - I2C_SCL_HELD_LOW_AFTER_READ 4, SCL held low beyond slave clock stretch time
*/
/**************************************************************************/
uint8_t twi_status()
{
  if (twi_write_start() != I2C_OK) return I2C_SDA_HELD_LOW_AFTER_INIT;
  if (twi_write_stop()  != I2C_OK) return I2C_SCL_HELD_LOW_AFTER_READ;
  if (SDA_READ() == LOW)           return I2C_SDA_HELD_LOW;
  if (SCL_READ() == LOW)           return I2C_SCL_HELD_LOW; 

  return I2C_OK;                                                       //OK!!!
}
