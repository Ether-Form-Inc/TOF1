// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#pragma once

// The VL53L1CB time of flight sensor on I2C2 (address 0x29), driven through ST's ultra low power API.
// Nothing here blocks for long: a failed call returns false and the caller tries again later.

#include <Tof_canard_messages.h>
#include <Tof_config.h>

#include <stm32g0xx_hal.h>
#include <stm32g0xx_hal_gpio.h>
#include <stm32g0xx_hal_i2c.h>

extern "C"
{
#include <VL53L1X_api.h>
#include <vl53l1_platform.h>
}

#include <cstdint>

class Stm_tof
{
public:
  // Sets up the I2C peripheral and holds the sensor in reset (XSHUT low, PB2). Does not talk to the sensor.
  // Returns false if the peripheral itself could not be set up.
  bool init()
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOB, &gpio);

    // 100 kHz with the 16 MHz clock. The board has no external pull-ups (the pins' internal ones are used),
    // which is too weak for 400 kHz.
    i2c.Instance = I2C2;
    i2c.Init.Timing = 0x00503D58;
    i2c.Init.OwnAddress1 = 0;
    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c.Init.OwnAddress2 = 0;
    i2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    peripheral_ok = HAL_I2C_Init(&i2c) == HAL_OK &&
                    HAL_I2CEx_ConfigAnalogFilter(&i2c, I2C_ANALOGFILTER_ENABLE) == HAL_OK &&
                    HAL_I2CEx_ConfigDigitalFilter(&i2c, 0) == HAL_OK;
    VL53L1_PlatformSetI2C(&i2c);
    return peripheral_ok;
  }

  // One attempt to bring the sensor up: resets it with XSHUT, waits for it to boot, loads its configuration and
  // starts ranging. Call repeatedly until it succeeds. Takes about 70 ms.
  bool connect(uint8_t mode, uint16_t period_ms)
  {
    if (!peripheral_ok)
    {
      return false;
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

    // The sensor needs about 1.2 ms to boot after XSHUT is released
    uint8_t booted = 0;
    for (int i = 0; i < 20 && !booted; i++)
    {
      HAL_Delay(1);
      uint8_t state = 0;
      if (VL53L1X_BootState(address, &state, &i2c) == 0 && (state & 1))
      {
        booted = 1;
      }
    }
    if (!booted || VL53L1X_SensorInit(address, &i2c) != 0)
    {
      return false;
    }
    return configure(mode, period_ms);
  }

  // Applies a new distance mode and measurement period to a sensor that is already ranging
  bool reconfigure(uint8_t mode, uint16_t period_ms)
  {
    VL53L1X_StopRanging(address, &i2c);
    return configure(mode, period_ms);
  }

  // Returns true and fills the sample if a new measurement was waiting
  bool read(Tof_canard::Sample &sample)
  {
    uint8_t ready = 0;
    if (VL53L1X_CheckForDataReady(address, &ready, &i2c) != 0 || !ready)
    {
      return false;
    }
    VL53L1X_Result_t result;
    const bool ok = VL53L1X_GetResult(address, &result, &i2c) == 0;
    VL53L1X_ClearInterrupt(address, &i2c); // arms the next measurement even if the read failed
    if (!ok)
    {
      return false;
    }
    sample.range_status = result.Status;
    sample.distance_mm = result.Distance;
    sample.mode = current_mode;
    return true;
  }

private:
  static constexpr uint16_t address = 0x29; // 7 bit address (0x52 as the sensor's documentation writes it)

  bool configure(uint8_t mode, uint16_t period_ms)
  {
    // The timing budget has to be set after the distance mode, which resets it
    const uint16_t budget = Tof_config::timing_budget_ms(period_ms);
    bool ok = VL53L1X_SetDistanceMode(address, mode, &i2c) == 0;
    ok = ok && VL53L1X_SetTimingBudgetInMs(address, budget, &i2c) == 0;
    ok = ok && VL53L1X_SetInterMeasurementInMs(address, period_ms, &i2c) == 0;
    ok = ok && VL53L1X_StartRanging(address, &i2c) == 0;
    if (ok)
    {
      current_mode = mode;
    }
    return ok;
  }

  I2C_HandleTypeDef i2c{};
  bool peripheral_ok{false};
  uint8_t current_mode{Tof_config::default_mode};
};
