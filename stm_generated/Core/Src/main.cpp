// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#include "main.h"

#include <Canard_tof_node.h>
#include <Stm_settings.h>
#include <Tof_config.h>

#include <stdint.h>

// Replaces libstdc++'s default terminate handler, which would link in the C++ symbol
// demangler (about 32 KB of flash) just to print a message nobody can see.
namespace __gnu_cxx
{
void __verbose_terminate_handler()
{
  __disable_irq();
  while (1)
  {
  }
}
} // namespace __gnu_cxx

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock
  SystemClock_Config();

  // Initialize all configured peripherals
  MX_GPIO_Init();

  // The ID jumpers on J3 form a 4 bit number. The pins have external pull-ups, so a fitted jumper
  // (shorted to GND) reads low and sets its bit. GPIO1 (PB5) is the LSB, then GPIO2 (PB4), GPIO3 (PB3)
  // and GPIO4 (PA15). The node ID is that number plus Tof_config::node_id_offset, so with no jumpers
  // fitted it is the offset itself.
  uint8_t jumpers = 0;
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET)
  {
    jumpers |= (1 << 0);
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET)
  {
    jumpers |= (1 << 1);
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET)
  {
    jumpers |= (1 << 2);
  }
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)
  {
    jumpers |= (1 << 3);
  }

  // A node ID stored in flash overrides the jumpers (see Tof_config::resolve_node_id)
  uint8_t stored;
  const bool have_stored_id = Stm_settings::read(Stm_settings::node_id, stored);
  const uint8_t can_id = Tof_config::resolve_node_id(jumpers, have_stored_id, stored);

  uint8_t value;
  const bool have_mode = Stm_settings::read(Stm_settings::distance_mode, value);
  const uint8_t mode = Tof_config::resolve_mode(have_mode, value);
  const bool have_period = Stm_settings::read(Stm_settings::update_rate, value);
  const uint16_t period_ms = Tof_config::resolve_period_ms(have_period, value);

  const bool have_orientation = Stm_settings::read(Stm_settings::beam_orientation, value);
  const uint8_t orientation = Tof_config::resolve_orientation(have_orientation, value);

  // This needs to be static as we will blow through the stack otherwise
  static Canard_tof_node node{can_id, mode, period_ms, orientation};

  // Serves the CAN node and the sensor forever
  node.run();

  // This should never be reached
  while (1)
  {
    Error_Handler();
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Hardware ID pins J3: PB3 PB4 PB5 (GPIO3 GPIO2 GPIO1) and PA15 (GPIO4). They have external pull-ups. */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
