/**
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/**
 * @file  vl53l1x_calibration.c
 * @brief Calibration functions implementation
 */
#include "VL53L1X_calibration.h"
#include "VL53L1X_api.h"

#define ALGO__PART_TO_PART_RANGE_OFFSET_MM 0x1E00
#define MM_CONFIG__INNER_OFFSET_MM 0x2000
#define MM_CONFIG__OUTER_OFFSET_MM 0x2200

int8_t VL53L1X_CalibrateOffset(uint16_t dev, uint16_t TargetDistInMm, int16_t *offset, I2C_HandleTypeDef *hi2c2)
{
  uint8_t i, tmp;
  int16_t AverageDistance = 0;
  uint16_t distance;
  VL53L1X_ERROR status = 0;

  status |= VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0, hi2c2);
  status |= VL53L1_WrWord(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0, hi2c2);
  status |= VL53L1_WrWord(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0, hi2c2);
  status |= VL53L1X_StartRanging(dev, hi2c2); /* Enable VL53L1X sensor */
  for (i = 0; i < 50; i++)
  {
    tmp = 0;
    while (tmp == 0)
    {
      status |= VL53L1X_CheckForDataReady(dev, &tmp, hi2c2);
    }
    status |= VL53L1X_GetDistance(dev, &distance, hi2c2);
    status |= VL53L1X_ClearInterrupt(dev, hi2c2);
    AverageDistance = AverageDistance + distance;
  }
  status |= VL53L1X_StopRanging(dev, hi2c2);
  AverageDistance = AverageDistance / 50;
  *offset = TargetDistInMm - AverageDistance;
  status |= VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, *offset * 4, hi2c2);
  return status;
}

int8_t VL53L1X_CalibrateXtalk(uint16_t dev, uint16_t TargetDistInMm, uint16_t *xtalk, I2C_HandleTypeDef *hi2c2)
{
  uint8_t i, tmp;
  float AverageSignalRate = 0;
  float AverageDistance = 0;
  float AverageSpadNb = 0;
  uint16_t distance = 0, spadNum;
  uint16_t sr;
  VL53L1X_ERROR status = 0;
  uint32_t calXtalk;

  status |= VL53L1_WrWord(dev, 0x0016, 0, hi2c2);
  status |= VL53L1X_StartRanging(dev, hi2c2);
  for (i = 0; i < 50; i++)
  {
    tmp = 0;
    while (tmp == 0)
    {
      status |= VL53L1X_CheckForDataReady(dev, &tmp, hi2c2);
    }
    status |= VL53L1X_GetSignalRate(dev, &sr, hi2c2);
    status |= VL53L1X_GetDistance(dev, &distance, hi2c2);
    status |= VL53L1X_ClearInterrupt(dev, hi2c2);
    AverageDistance = AverageDistance + distance;
    status = VL53L1X_GetSpadNb(dev, &spadNum, hi2c2);
    AverageSpadNb = AverageSpadNb + spadNum;
    AverageSignalRate = AverageSignalRate + sr;
  }
  status |= VL53L1X_StopRanging(dev, hi2c2);
  AverageDistance = AverageDistance / 50;
  AverageSpadNb = AverageSpadNb / 50;
  AverageSignalRate = AverageSignalRate / 50;
  /* Calculate Xtalk value */
  calXtalk = (uint16_t)(512 * (AverageSignalRate * (1 - (AverageDistance / TargetDistInMm))) / AverageSpadNb);
  if (calXtalk > 0xffff)
    calXtalk = 0xffff;
  *xtalk = (uint16_t)((calXtalk * 1000) >> 9);
  status |= VL53L1_WrWord(dev, 0x0016, (uint16_t)calXtalk, hi2c2);
  return status;
}
