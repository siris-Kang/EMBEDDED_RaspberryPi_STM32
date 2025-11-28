/*
 * motor.c
 *
 *  Created on: Nov 27, 2025
 *      Author: SSAFY
 */


#include "motor.h"
#include "gpio.h"

void Motor_Init(void)
{
  HAL_GPIO_WritePin(GPIOC, Motor_ENA_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, Motor_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, Motor_IN2_Pin, GPIO_PIN_RESET);
}

void Motor_SetDirection(MotorDir_t dir)
{
  switch (dir)
  {
    case MOTOR_DIR_FORWARD:
      HAL_GPIO_WritePin(GPIOC, Motor_ENA_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, Motor_IN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, Motor_IN2_Pin, GPIO_PIN_RESET);
      break;

    case MOTOR_DIR_BACKWARD:
      HAL_GPIO_WritePin(GPIOC, Motor_ENA_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, Motor_IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, Motor_IN2_Pin, GPIO_PIN_SET);
      break;

    case MOTOR_DIR_STOP:
    default:
      HAL_GPIO_WritePin(GPIOC, Motor_ENA_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, Motor_IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, Motor_IN2_Pin, GPIO_PIN_RESET);
      break;
  }
}
