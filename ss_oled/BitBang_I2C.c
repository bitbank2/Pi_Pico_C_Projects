//
// Bit Bang I2C library
// Copyright (c) 2018 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
// Project started 10/12/2018
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "BitBang_I2C.h"

static uint8_t SDA_READ(uint8_t iSDA)
{
    return gpio_get(iSDA);
}
static void SCL_HIGH(uint8_t iSCL)
{
    gpio_init(iSCL);
    gpio_set_dir(iSCL, GPIO_IN);
}

static void SCL_LOW(uint8_t iSCL)
{
    gpio_set_dir(iSCL, GPIO_OUT);
    gpio_put(iSCL, LOW);
}

static void SDA_HIGH(uint8_t iSDA)
{
    gpio_init(iSDA);
    gpio_set_dir(iSDA, GPIO_IN);
}

static void SDA_LOW(uint8_t iSDA)
{
    gpio_set_dir(iSDA, GPIO_OUT);
    gpio_put(iSDA, LOW);
}

//
// Transmit a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//

static int i2cByteOut(BBI2C *pI2C, uint8_t b)
{
uint8_t i, ack;
uint8_t iSDA = pI2C->iSDA;
uint8_t iSCL = pI2C->iSCL; // in case of bad C compiler
int iDelay = pI2C->iDelay;

  for (i=0; i<8; i++)
  {
      if (b & 0x80)
        SDA_HIGH(iSDA); // set data line to 1
      else
        SDA_LOW(iSDA); // set data line to 0
      SCL_HIGH(iSCL); // clock high (slave latches data)
      sleep_us(iDelay);
      SCL_LOW(iSCL); // clock low
      b <<= 1;
      sleep_us(iDelay);
  } // for i
// read ack bit
  SDA_HIGH(iSDA); // set data line for reading
  SCL_HIGH(iSCL); // clock line high
  sleep_us(iDelay); // DEBUG - delay/2
  ack = SDA_READ(iSDA);
  SCL_LOW(iSCL); // clock low
  sleep_us(iDelay); // DEBUG - delay/2
  SDA_LOW(iSDA); // data low
  return (ack == 0) ? 1:0; // a low ACK bit means success
} /* i2cByteOut() */

static int i2cByteOutFast(BBI2C *pI2C, uint8_t b)
{
uint8_t i, ack, iSDA, iSCL;
int iDelay;

     iSDA = pI2C->iSDA;
     iSCL = pI2C->iSCL;
     iDelay = pI2C->iDelay;

     if (b & 0x80)
        SDA_HIGH(iSDA); // set data line to 1
     else
        SDA_LOW(iSDA); // set data line to 0
     for (i=0; i<8; i++)
     {
         SCL_HIGH(iSCL); // clock high (slave latches data)
         sleep_us(iDelay);
         SCL_LOW(iSCL); // clock low
         sleep_us(iDelay);
     } // for i
// read ack bit
  SDA_HIGH(iSDA); // set data line for reading
  SCL_HIGH(iSCL); // clock line high
  sleep_us(pI2C->iDelay); // DEBUG - delay/2
  ack = SDA_READ(iSDA);
  SCL_LOW(iSCL); // clock low
  sleep_us(pI2C->iDelay); // DEBUG - delay/2
  SDA_LOW(iSDA); // data low
  return (ack == 0) ? 1:0; // a low ACK bit means success
} /* i2cByteOutFast() */
//
// Receive a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//
static uint8_t i2cByteIn(BBI2C *pI2C, uint8_t bLast)
{
uint8_t i;
uint8_t b = 0;

     SDA_HIGH(pI2C->iSDA); // set data line as input
     for (i=0; i<8; i++)
     {
         sleep_us(pI2C->iDelay); // wait for data to settle
         SCL_HIGH(pI2C->iSCL); // clock high (slave latches data)
         b <<= 1;
         if (SDA_READ(pI2C->iSDA) != 0) // read the data bit
           b |= 1; // set data bit
         SCL_LOW(pI2C->iSCL); // cloc low
     } // for i
     if (bLast)
        SDA_HIGH(pI2C->iSDA); // last byte sends a NACK
     else
        SDA_LOW(pI2C->iSDA);
     SCL_HIGH(pI2C->iSCL); // clock high
     sleep_us(pI2C->iDelay);
     SCL_LOW(pI2C->iSCL); // clock low to send ack
     sleep_us(pI2C->iDelay);
     SDA_LOW(pI2C->iSDA); // data low
  return b;
} /* i2cByteIn() */

//
// Send I2C STOP condition
//
static void i2cEnd(BBI2C *pI2C)
{
   SDA_LOW(pI2C->iSDA); // data line low
   sleep_us(pI2C->iDelay);
   SCL_HIGH(pI2C->iSCL); // clock high
   sleep_us(pI2C->iDelay);
   SDA_HIGH(pI2C->iSDA); // data high
   sleep_us(pI2C->iDelay);
} /* i2cEnd() */


static int i2cBegin(BBI2C *pI2C, uint8_t addr, uint8_t bRead)
{
   int rc;
   SDA_LOW(pI2C->iSDA); // data line low first
   sleep_us(pI2C->iDelay);
   SCL_LOW(pI2C->iSCL); // then clock line low is a START signal
   addr <<= 1;
   if (bRead)
      addr++; // set read bit
   rc = i2cByteOut(pI2C, addr); // send the slave address and R/W bit
   return rc;
} /* i2cBegin() */

static int i2cWrite(BBI2C *pI2C, uint8_t *pData, int iLen)
{
uint8_t b;
int rc, iOldLen = iLen;

   rc = 1;
   while (iLen && rc == 1)
   {
      b = *pData++;
//      if (b == 0xff || b == 0)
//         rc = i2cByteOutFast(pI2C, b); // speed it up a bit more if all bits are ==
//      else
         rc = i2cByteOut(pI2C, b);
      if (rc == 1) // success
      {
         iLen--;
      }
   } // for each byte
   return (rc == 1) ? (iOldLen - iLen) : 0; // 0 indicates bad ack from sending a byte
} /* i2cWrite() */

static void i2cRead(BBI2C *pI2C, uint8_t *pData, int iLen)
{
   while (iLen--)
   {
      *pData++ = i2cByteIn(pI2C, iLen == 0);
   } // for each byte
} /* i2cRead() */
//
// Initialize the I2C BitBang library
// Pass the pin numbers used for SDA and SCL
// as well as the clock rate in Hz
//
void I2CInit(BBI2C *pI2C, uint32_t iClock)
{
   if (pI2C == NULL) return;

   if (pI2C->bWire) // use Wire library
   {
      i2c_init(i2c0, iClock);
      gpio_set_function(pI2C->iSDA, GPIO_FUNC_I2C);
      gpio_set_function(pI2C->iSCL, GPIO_FUNC_I2C);
      gpio_pull_up(pI2C->iSDA);
      gpio_pull_up(pI2C->iSCL);
      return;
   }
   if (pI2C->iSDA < 0xa0)
   {
     gpio_init(pI2C->iSDA);
     gpio_init(pI2C->iSCL);
//     gpio_set_dir(pI2C->iSDA, GPIO_OUT);
//     gpio_set_dir(pI2C->iSCL, GPIO_OUT);
//     gpio_put(pI2C->iSDA, LOW); // setting low = enabling as outputs
//     gpio_put(pI2C->iSCL, LOW);
     gpio_set_dir(pI2C->iSDA, GPIO_IN); // let the lines float (tri-state)
     gpio_set_dir(pI2C->iSCL, GPIO_IN);

   }
  // For now, we only support 100, 400 or 800K clock rates
  // all other values default to 100K
   if (iClock >= 1000000)
      pI2C->iDelay = 0; // the code execution is enough delay
   else if (iClock >= 800000)
      pI2C->iDelay = 1;
   else if (iClock >= 400000)
      pI2C->iDelay = 2;
   else if (iClock >= 100000)
      pI2C->iDelay = 10;
   else pI2C->iDelay = (uint16_t)(1000000 / iClock);
} /* i2cInit() */
//
// Test a specific I2C address to see if a device responds
// returns 0 for no response, 1 for a response
//
uint8_t I2CTest(BBI2C *pI2C, uint8_t addr)
{
uint8_t response = 0;

  if (pI2C->bWire)
  {
     int ret;
     uint8_t rxdata;
     ret = i2c_read_blocking(i2c0, addr, &rxdata, 1, false);
     return (ret >= 0);
  }
  if (i2cBegin(pI2C, addr, 0)) // try to write to the given address
  {
    response = 1;
  }
  i2cEnd(pI2C);
  return response;
} /* I2CTest() */
//
// Scans for I2C devices on the bus
// returns a bitmap of devices which are present (128 bits = 16 bytes, LSB first)
// A set bit indicates that a device responded at that address
//
void I2CScan(BBI2C *pI2C, uint8_t *pMap)
{
  int i;
  for (i=0; i<16; i++) // clear the bitmap
    pMap[i] = 0;
  for (i=1; i<128; i++) // try every address
  {
    if (I2CTest(pI2C, i))
    {
      pMap[i >> 3] |= (1 << (i & 7));
    }
  }
} /* I2CScan() */
//
// Write I2C data
// quits if a NACK is received and returns 0
// otherwise returns the number of bytes written
//
int I2CWrite(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen)
{
  int rc = 0;
  
  if (pI2C->bWire)
  {
    rc = i2c_write_blocking(i2c0, iAddr, pData, iLen, true); // true to keep master control of bus
    return rc >= 0 ? iLen : 0;
  }
  rc = i2cBegin(pI2C, iAddr, 0);
  if (rc == 1) // slave sent ACK for its address
  {
     rc = i2cWrite(pI2C, pData, iLen);
  }
  i2cEnd(pI2C);
  return rc; // returns the number of bytes sent or 0 for error
} /* I2CWrite() */
//
// Read N bytes starting at a specific I2C internal register
//
int I2CReadRegister(BBI2C *pI2C, uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen)
{
  int rc;
  
  if (pI2C->bWire) // use the wire library
  {
      rc = i2c_write_blocking(i2c0, iAddr, &u8Register, 1, true); // true to keep master control of bus 
      if (rc >= 0) {
         rc = i2c_read_blocking(i2c0, iAddr, pData, iLen, false);
      }
      return (rc >= 0);
  }
  rc = i2cBegin(pI2C, iAddr, 0); // start a write operation
  if (rc == 1) // slave sent ACK for its address
  {
     rc = i2cWrite(pI2C, &u8Register, 1); // write the register we want to read from
     if (rc == 1)
     {
       i2cEnd(pI2C);
       rc = i2cBegin(pI2C, iAddr, 1); // start a read operation
       if (rc == 1)
       {
         i2cRead(pI2C, pData, iLen);
       }
     }
  }
  i2cEnd(pI2C);
  return rc; // returns 1 for success, 0 for error
} /* I2CReadRegister() */
//
// Read N bytes
//
int I2CRead(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen)
{
  int rc;
  
    if (pI2C->bWire) // use the wire library
    {
       rc = i2c_read_blocking(i2c0, iAddr, pData, iLen, false);
       return (rc >= 0);
    }
  rc = i2cBegin(pI2C, iAddr, 1);
  if (rc == 1) // slave sent ACK for its address
  {
     i2cRead(pI2C, pData, iLen);
  }
  i2cEnd(pI2C);
  return rc; // returns 1 for success, 0 for error
} /* I2CRead() */
//
// Figure out what device is at that address
// returns the enumerated value
//
int I2CDiscoverDevice(BBI2C *pI2C, uint8_t i)
{
uint8_t j, cTemp[8];
int iDevice = DEVICE_UNKNOWN;

  if (i == 0x3c || i == 0x3d) // Probably an OLED display
  {
    I2CReadRegister(pI2C, i, 0x00, cTemp, 1);
    cTemp[0] &= 0xbf; // mask off power on/off bit
    if (cTemp[0] == 0x8) // SH1106
       iDevice = DEVICE_SH1106;
    else if (cTemp[0] == 3 || cTemp[0] == 6)
       iDevice = DEVICE_SSD1306;
    return iDevice;
  }
  
  if (i == 0x34 || i == 0x35) // Probably an AXP202/AXP192 PMU chip
  {
    I2CReadRegister(pI2C, i, 0x03, cTemp, 1); // chip ID
    if (cTemp[0] == 0x41)
       return DEVICE_AXP202;
    else if (cTemp[0] == 0x03)
       return DEVICE_AXP192;
  }
  
  if (i >= 0x40 && i <= 0x4f) // check for TI INA219 power measurement sensor
  {
    I2CReadRegister(pI2C, i, 0x00, cTemp, 2);
    if (cTemp[0] == 0x39 && cTemp[1] == 0x9f)
       return DEVICE_INA219;
  }
  
  // Check for Microchip 24AAXXXE64 family serial 2 Kbit EEPROM
  if (i >= 0x50 && i <= 0x57) {
    uint32_t u32Temp = 0;
    I2CReadRegister(pI2C, i, 0xf8, (uint8_t *)&u32Temp,
                    3); // check for Microchip's OUI
    if (u32Temp == 0x000004a3 || u32Temp == 0x00001ec0 ||
        u32Temp == 0x00d88039 || u32Temp == 0x005410ec)
      return DEVICE_24AAXXXE64;
  }
  
//  else if (i == 0x5b) // MLX90615?
//  {
//    I2CReadRegister(pI2C, i, 0x10, cTemp, 3);
//    for (j=0; j<3; j++) Serial.println(cTemp[j], HEX);
//  }
  // try to identify it from the known devices using register contents
  {    
    // Check for TI HDC1080
    I2CReadRegister(pI2C, i, 0xff, cTemp, 2);
    if (cTemp[0] == 0x10 && cTemp[1] == 0x50)
       return DEVICE_HDC1080;

    // Check for BME680
    if (i == 0x76 || i == 0x77)
    {
       I2CReadRegister(pI2C, i, 0xd0, cTemp, 1); // chip ID
       if (cTemp[0] == 0x61) // BME680
          return DEVICE_BME680;
    }
    // Check for VL53L0X
    I2CReadRegister(pI2C, i, 0xc0, cTemp, 3);
    if (cTemp[0] == 0xee && cTemp[1] == 0xaa && cTemp[2] == 0x10)
       return DEVICE_VL53L0X;

    // Check for CCS811
    I2CReadRegister(pI2C, i, 0x20, cTemp, 1);
    if (cTemp[0] == 0x81) // Device ID
       return DEVICE_CCS811;

    // Check for LIS3DSH accelerometer from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0x3f) // WHO_AM_I
       return DEVICE_LIS3DSH;

    // Check for LIS3DH accelerometer from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0x33) // WHO_AM_I
       return DEVICE_LIS3DH;

    // Check for LSM9DS1 magnetometer/gyro/accel sensor from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0x68) // WHO_AM_I
       return DEVICE_LSM9DS1;

    // Check for LPS25H pressure sensor from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0xbd) // WHO_AM_I
       return DEVICE_LPS25H;
    
    // Check for HTS221 temp/humidity sensor from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0xbc) // WHO_AM_I
       return DEVICE_HTS221;
    
    // Check for MAG3110
    I2CReadRegister(pI2C, i, 0x07, cTemp, 1);
    if (cTemp[0] == 0xc4) // WHO_AM_I
       return DEVICE_MAG3110;

    // Check for LM8330 keyboard controller
    I2CReadRegister(pI2C, i, 0x80, cTemp, 2);
    if (cTemp[0] == 0x0 && cTemp[1] == 0x84) // manufacturer code + software revision
       return DEVICE_LM8330;

    // Check for MAX44009
    if (i == 0x4a || i == 0x4b)
    {
      for (j=0; j<8; j++)
        I2CReadRegister(pI2C, i, j, &cTemp[j], 1); // check for power-up reset state of registers
      if ((cTemp[2] == 3 || cTemp[2] == 2) && cTemp[6] == 0 && cTemp[7] == 0xff)
         return DEVICE_MAX44009;
    }
       
    // Check for ADS1115
    I2CReadRegister(pI2C, i, 0x02, cTemp, 2); // Lo_thresh defaults to 0x8000
    I2CReadRegister(pI2C, i, 0x03, &cTemp[2], 2); // Hi_thresh defaults to 0x7fff
    if (cTemp[0] == 0x80 && cTemp[1] == 0x00 && cTemp[2] == 0x7f && cTemp[3] == 0xff)
       return DEVICE_ADS1115;

    // Check for MCP9808
    I2CReadRegister(pI2C, i, 0x06, cTemp, 2); // manufacturer ID && get device ID/revision
    I2CReadRegister(pI2C, i, 0x07, &cTemp[2], 2); // need to read them individually
    if (cTemp[0] == 0 && cTemp[1] == 0x54 && cTemp[2] == 0x04 && cTemp[3] == 0x00)
       return DEVICE_MCP9808;
       
    // Check for BMP280/BME280
    I2CReadRegister(pI2C, i, 0xd0, cTemp, 1);
    if (cTemp[0] == 0x55) // BMP180
       return DEVICE_BMP180;
    else if (cTemp[0] == 0x58)
       return DEVICE_BMP280;
    else if (cTemp[0] == 0x60) // BME280
       return DEVICE_BME280;

    // Check for LSM6DS3
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1); // WHO_AM_I
    if (cTemp[0] == 0x69)
       return DEVICE_LSM6DS3;
       
    // Check for ADXL345
    I2CReadRegister(pI2C, i, 0x00, cTemp, 1); // DEVID
    if (cTemp[0] == 0xe5)
       return DEVICE_ADXL345;
       
    // Check for MPU-60x0i, MPU-688x, and MPU-9250
    I2CReadRegister(pI2C, i, 0x75, cTemp, 1);
    if (cTemp[0] == (i & 0xfe)) // Current I2C address (low bit set to 0)
       return DEVICE_MPU6000;
    else if (cTemp[0] == 0x71)
       return DEVICE_MPU9250;
    else if (cTemp[0] == 0x19)
       return DEVICE_MPU6886;

    // Check for DS3231 RTC
    I2CReadRegister(pI2C, i, 0x0e, cTemp, 1); // read the control register
    if (i == 0x68 &&
        cTemp[0] == 0x1c) // fixed I2C address and power on reset value
      return DEVICE_DS3231;

    // Check for DS1307 RTC
    I2CReadRegister(pI2C, i, 0x07, cTemp, 1); // read the control register
    if (i == 0x68 &&
        cTemp[0] == 0x03) // fixed I2C address and power on reset value
      return DEVICE_DS1307;
        
  }
  return iDevice;
} /* I2CDiscoverDevice() */
