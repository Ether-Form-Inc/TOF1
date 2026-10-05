// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
//
// Platform layer of the ST VL53L1X ULD API for the STM32G0 HAL. The ULD API calls these functions to reach the
// sensor's 16 bit registers over I2C (big endian on the wire). Some of them receive the I2C handle and some do not,
// so the handle is also kept here: set it once with VL53L1_PlatformSetI2C().
#include <vl53l1_platform.h>

#define VL53L1_I2C_TIMEOUT_MS 100

static I2C_HandleTypeDef *platform_i2c;

void VL53L1_PlatformSetI2C(I2C_HandleTypeDef *hi2c)
{
  platform_i2c = hi2c;
}

static int8_t mem_write(I2C_HandleTypeDef *hi2c, uint16_t dev, uint16_t index, uint8_t *data, uint16_t size)
{
  return HAL_I2C_Mem_Write(hi2c, (uint16_t)(dev << 1), index, I2C_MEMADD_SIZE_16BIT, data, size,
                           VL53L1_I2C_TIMEOUT_MS) == HAL_OK ? 0 : 1;
}

static int8_t mem_read(I2C_HandleTypeDef *hi2c, uint16_t dev, uint16_t index, uint8_t *data, uint16_t size)
{
  return HAL_I2C_Mem_Read(hi2c, (uint16_t)(dev << 1), index, I2C_MEMADD_SIZE_16BIT, data, size,
                          VL53L1_I2C_TIMEOUT_MS) == HAL_OK ? 0 : 1;
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  return mem_write(platform_i2c, dev, index, pdata, (uint16_t)count);
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  return mem_read(platform_i2c, dev, index, pdata, (uint16_t)count);
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data, I2C_HandleTypeDef *hi2c)
{
  return mem_write(hi2c, dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data, I2C_HandleTypeDef *hi2c)
{
  uint8_t buffer[2] = {(uint8_t)(data >> 8), (uint8_t)data};
  return mem_write(hi2c, dev, index, buffer, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
  uint8_t buffer[4] = {(uint8_t)(data >> 24), (uint8_t)(data >> 16), (uint8_t)(data >> 8), (uint8_t)data};
  return mem_write(platform_i2c, dev, index, buffer, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data, I2C_HandleTypeDef *hi2c)
{
  return mem_read(hi2c, dev, index, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data, I2C_HandleTypeDef *hi2c)
{
  uint8_t buffer[2];
  if (mem_read(hi2c, dev, index, buffer, 2) != 0)
  {
    return 1;
  }
  *data = (uint16_t)((buffer[0] << 8) | buffer[1]);
  return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
  uint8_t buffer[4];
  if (mem_read(platform_i2c, dev, index, buffer, 4) != 0)
  {
    return 1;
  }
  *data = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | buffer[3];
  return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
  (void)dev;
  HAL_Delay((uint32_t)wait_ms);
  return 0;
}
