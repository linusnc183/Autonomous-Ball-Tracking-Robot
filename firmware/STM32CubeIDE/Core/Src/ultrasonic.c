#include "ultrasonic.h"
#include "main.h"

static float last_distance_cm = -1.0f;

// ECHO isn't on a timer capture pin, so time it with the DWT cycle counter
static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t cycles_to_us(uint32_t cycles)
{
  return cycles / (SystemCoreClock / 1000000U);
}

void Ultrasonic_Init(void)
{
  DWT_Init();
}

void Ultrasonic_Update(void)
{
  uint32_t t0;

  HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin, GPIO_PIN_RESET);
  t0 = DWT->CYCCNT;
  while (cycles_to_us(DWT->CYCCNT - t0) < 2) { }

  HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin, GPIO_PIN_SET);
  t0 = DWT->CYCCNT;
  while (cycles_to_us(DWT->CYCCNT - t0) < 10) { }
  HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin, GPIO_PIN_RESET);

  uint32_t wait_start = DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin) == GPIO_PIN_RESET)
  {
    if (cycles_to_us(DWT->CYCCNT - wait_start) > 30000) return;
  }

  uint32_t echo_start = DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin) == GPIO_PIN_SET)
  {
    if (cycles_to_us(DWT->CYCCNT - echo_start) > 30000) return;
  }

  uint32_t pulse_us = cycles_to_us(DWT->CYCCNT - echo_start);
  last_distance_cm = (float)pulse_us / 58.2f;  // ~58.2us per cm round-trip
}

float Ultrasonic_GetDistanceCm(void)
{
  return last_distance_cm;
}
