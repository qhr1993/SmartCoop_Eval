/**************************************************************************/
/*!
    @file     Adafruit_TSL2591.cpp
    @author   KT0WN (adafruit.com)

    This is a library for the Adafruit TSL2591 breakout board
    This library works with the Adafruit TSL2591 breakout
    ----> https://www.adafruit.com/products/1980

    Check out the links above for our tutorials and wiring diagrams
    These chips use I2C to communicate

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2014 Adafruit Industries
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#include <stdlib.h>
#include "TSL2591.h"
#include "main.h"


static uint8_t _initialized = 0;
static tsl2591IntegrationTime_t _integration = TSL2591_INTEGRATIONTIME_100MS;
static tsl2591Gain_t _gain       = TSL2591_GAIN_MED;

uint8_t TSL2591_begin(I2C_HandleTypeDef *hi2c)
{
  uint8_t id = TSL2591_read8(hi2c, TSL2591_COMMAND_BIT | TSL2591_REGISTER_DEVICE_ID);
  if (id == 0x50 )
  {
     uasrt_logging_printf("Found Adafruit_TSL2591...\r\n");
  }
  else
  {
    uasrt_logging_printf("NOT Found Adafruit_TSL2591, returned 0x%02X\r\n", id);
    return 0;
  }

  _initialized = 1;

  // Set default integration time and gain
  TSL2591_setTiming(hi2c, _integration);
  TSL2591_setGain(hi2c, _gain);

  // Note: by default, the device is in power down mode on bootup
  TSL2591_disable(hi2c);

  return 1;
}

void TSL2591_enable(I2C_HandleTypeDef *hi2c)
{
  if (!_initialized)
  {
    if (!TSL2591_begin(hi2c))
    {
      return;
    }
  }

  // Enable the device by setting the control bit to 0x01
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE, TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN | TSL2591_ENABLE_NPIEN);
}

void TSL2591_disable(I2C_HandleTypeDef *hi2c)
{
  if (!_initialized)
  {
    if (!TSL2591_begin(hi2c))
    {
      return;
    }
  }

  // Disable the device by setting the control bit to 0x00
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE, TSL2591_ENABLE_POWEROFF);
}

void TSL2591_setGain(I2C_HandleTypeDef *hi2c, tsl2591Gain_t gain)
{
  if (!_initialized)
  {
    if (!TSL2591_begin(hi2c))
    {
      return;
    }
  }

  TSL2591_enable(hi2c);
  _gain = gain;
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, _integration | _gain);
  TSL2591_disable(hi2c);
}

tsl2591Gain_t TSL2591_getGain(I2C_HandleTypeDef *hi2c )
{
  return _gain;
}

void TSL2591_setTiming(I2C_HandleTypeDef *hi2c, tsl2591IntegrationTime_t integration)
{
  if (!_initialized)
  {
    if (!TSL2591_begin(hi2c))
    {
      return;
    }
  }

  TSL2591_enable(hi2c);
  _integration = integration;
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, _integration | _gain);
  TSL2591_disable(hi2c);
}

tsl2591IntegrationTime_t TSL2591_getTiming(I2C_HandleTypeDef *hi2c)
{
  return _integration;
}

uint32_t TSL2591_calculateLux(I2C_HandleTypeDef *hi2c, uint16_t ch0, uint16_t ch1)
{
  float    atime, again;
  float    cpl, lux1, lux2, lux;
  uint32_t chan0, chan1;

  // Check for overflow conditions first
  if ((ch0 == 0xFFFF) | (ch1 == 0xFFFF))
  {
    // Signal an overflow
    return 0;
  }

  // Note: This algorithm is based on preliminary coefficients
  // provided by AMS and may need to be updated in the future

  switch (_integration)
  {
    case TSL2591_INTEGRATIONTIME_100MS :
      atime = 100.0F;
      break;
    case TSL2591_INTEGRATIONTIME_200MS :
      atime = 200.0F;
      break;
    case TSL2591_INTEGRATIONTIME_300MS :
      atime = 300.0F;
      break;
    case TSL2591_INTEGRATIONTIME_400MS :
      atime = 400.0F;
      break;
    case TSL2591_INTEGRATIONTIME_500MS :
      atime = 500.0F;
      break;
    case TSL2591_INTEGRATIONTIME_600MS :
      atime = 600.0F;
      break;
    default: // 100ms
      atime = 100.0F;
      break;
  }

  switch (_gain)
  {
    case TSL2591_GAIN_LOW :
      again = 1.0F;
      break;
    case TSL2591_GAIN_MED :
      again = 25.0F;
      break;
    case TSL2591_GAIN_HIGH :
      again = 428.0F;
      break;
    case TSL2591_GAIN_MAX :
      again = 9876.0F;
      break;
    default:
      again = 1.0F;
      break;
  }

  // cpl = (ATIME * AGAIN) / DF
  cpl = (atime * again) / TSL2591_LUX_DF;

  lux1 = ( (float)ch0 - (TSL2591_LUX_COEFB * (float)ch1) ) / cpl;
  lux2 = ( ( TSL2591_LUX_COEFC * (float)ch0 ) - ( TSL2591_LUX_COEFD * (float)ch1 ) ) / cpl;
  lux = lux1 > lux2 ? lux1 : lux2;

  // Alternate lux calculation
  //lux = ( (float)ch0 - ( 1.7F * (float)ch1 ) ) / cpl;

  // Signal I2C had no errors
  return (uint32_t)lux;
}

uint32_t TSL2591_getFullLuminosity (I2C_HandleTypeDef *hi2c)
{
  if (!_initialized)
  {
    if (!TSL2591_begin(hi2c))
    {
      return 0;
    }
  }

  // Enable the device
  TSL2591_enable(hi2c);

  // Wait x ms for ADC to complete
  for (uint8_t d=0; d<=_integration; d++)
  {
    HAL_Delay(120);
  }

  uint32_t x;
  x = TSL2591_read16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_LOW);
  x <<= 16;
  x |= TSL2591_read16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW);

  TSL2591_disable(hi2c);

  return x;
}

uint16_t TSL2591_getLuminosity (I2C_HandleTypeDef *hi2c, uint8_t channel)
{
  uint32_t x = TSL2591_getFullLuminosity(hi2c);

  if (channel == TSL2591_FULLSPECTRUM)
  {
    // Reads two byte value from channel 0 (visible + infrared)
    return (x & 0xFFFF);
  }
  else if (channel == TSL2591_INFRARED)
  {
    // Reads two byte value from channel 1 (infrared)
    return (x >> 16);
  }
  else if (channel == TSL2591_VISIBLE)
  {
    // Reads all and subtracts out just the visible!
    return ( (x & 0xFFFF) - (x >> 16));
  }

  // unknown channel!
  return 0;
}

void TSL2591_registerInterrupt(I2C_HandleTypeDef *hi2c, uint16_t lowerThreshold, uint16_t upperThreshold)
{
  if (!_initialized)
  {
    if (!TSL2591_begin(hi2c))
    {
      return;
    }
  }

  TSL2591_enable(hi2c);
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_NPAILTL, lowerThreshold);
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_NPAILTH, lowerThreshold >> 8);
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_NPAIHTL, upperThreshold);
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_NPAIHTH, upperThreshold >> 8);
  TSL2591_disable(hi2c);
}

void TSL2591_registerInterruptPersist(I2C_HandleTypeDef *hi2c, uint16_t lowerThreshold, uint16_t upperThreshold, tsl2591Persist_t persist)
{
  if (!_initialized)
  {
    if (!TSL2591_begin(hi2c))
    {
      return;
    }
  }

  TSL2591_enable(hi2c);
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_PERSIST_FILTER,  persist);
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AILTL, lowerThreshold);
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AILTH, lowerThreshold >> 8);
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AIHTL, upperThreshold);
  TSL2591_write16(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AIHTH, upperThreshold >> 8);
  TSL2591_disable(hi2c);
}

void TSL2591_clearInterrupt(I2C_HandleTypeDef *hi2c)
{
  if (!_initialized)
  {
    if (!TSL2591_begin(hi2c))
    {
      return;
    }
  }

  TSL2591_enable(hi2c);
  TSL2591_write8(hi2c,TSL2591_CLEAR_INT);
  TSL2591_disable(hi2c);
}


uint8_t TSL2591_getStatus(I2C_HandleTypeDef *hi2c)
{
  if (!_initialized)
  {
    if (!TSL2591_begin(hi2c))
    {
      return 0;
    }
  }

  // Enable the device
  TSL2591_enable(hi2c);
  uint8_t x;
  x = TSL2591_read8(hi2c,TSL2591_COMMAND_BIT | TSL2591_REGISTER_DEVICE_STATUS);
  TSL2591_disable(hi2c);
  return x;
}


uint8_t TSL2591_read8(I2C_HandleTypeDef *hi2c, uint8_t reg)
{
  uint8_t x;
  uint8_t addr = reg;
  
  //uasrt_logging_printf("I2C Tx Status 0x%04X\r\n", HAL_I2C_Master_Transmit(hi2c,TSL2591_ADDR << 1, &addr, 1, 100));
  HAL_I2C_Master_Transmit(hi2c,TSL2591_ADDR << 1, &addr, 1, 100);
  //uasrt_logging_printf("I2C Tx: ADDR 0x%02X CMD 0x%02X\r\n",TSL2591_ADDR,addr);
  //uasrt_logging_printf("I2C Rx Status 0x%04X\r\n", HAL_I2C_Master_Receive(hi2c, TSL2591_ADDR << 1 , &x, 1, 100));
  HAL_I2C_Master_Receive(hi2c, TSL2591_ADDR << 1 , &x, 1, 100);
  //uasrt_logging_printf("I2C Rx: ADDR 0x%02X CMD 0x%02X\r\n",TSL2591_ADDR,x);
  return x;
}

uint16_t TSL2591_read16(I2C_HandleTypeDef *hi2c, uint8_t reg)
{
  uint8_t x[2];
  uint16_t r;
  uint8_t addr = reg;

  HAL_I2C_Master_Transmit(hi2c, TSL2591_ADDR << 1, &addr, 1, 100);
  HAL_I2C_Master_Receive(hi2c, TSL2591_ADDR << 1, x, 2, 100);

  r = x[0];
  r <<= 8;
  r |= x[1];
  return r;
}

void TSL2591_write16 (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
  uint8_t x[2];
  x[0] = reg;
  x[1] = value;

  HAL_I2C_Master_Transmit(hi2c, TSL2591_ADDR << 1, x, 2, 100);
}


void TSL2591_write8 (I2C_HandleTypeDef *hi2c, uint8_t reg)
{
  uint8_t x = reg; 
  HAL_I2C_Master_Transmit(hi2c, TSL2591_ADDR << 1, &x, 1, 100);
}
