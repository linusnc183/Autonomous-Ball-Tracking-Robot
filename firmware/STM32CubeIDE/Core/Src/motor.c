#include "motor.h"

extern TIM_HandleTypeDef htim1;

void Motor_Left(int16_t pwm)
{
  if (pwm > 1000) pwm = 1000;
  if (pwm < -1000) pwm = -1000;

  if (pwm >= 0)
  {
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_SET);
    pwm = -pwm;
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm * 65);
}

void Motor_Right(int16_t pwm)
{
  if (pwm > 1000) pwm = 1000;
  if (pwm < -1000) pwm = -1000;

  if (pwm >= 0)
  {
    HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_SET);
    pwm = -pwm;
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm * 65);
}
