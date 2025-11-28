/*
 * motor.c
 *
 *  Created on: Nov 27, 2025
 *      Author: SSAFY
 */


#include "motor.h"
#include "gpio.h"
#include "tim.h"

void Motor_Init(void)
{
  HAL_GPIO_WritePin(GPIOC, MOTOR_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, MOTOR_IN2_Pin, GPIO_PIN_RESET);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  Motor_SetSpeed(0);
}

void Motor_SetDirection(MotorDir_t dir)
{
  switch (dir)
  {
    case MOTOR_FORWARD:
      HAL_GPIO_WritePin(GPIOC, MOTOR_IN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, MOTOR_IN2_Pin, GPIO_PIN_RESET);
      break;

    case MOTOR_BACKWARD:
      HAL_GPIO_WritePin(GPIOC, MOTOR_IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, MOTOR_IN2_Pin, GPIO_PIN_SET);
      break;

    case MOTOR_STOP:
    default:
      HAL_GPIO_WritePin(GPIOC, MOTOR_IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, MOTOR_IN2_Pin, GPIO_PIN_RESET);
      Motor_SetSpeed(0);
      break;
  }
}

void Motor_SetSpeed(uint8_t level)   // 0~10
{
    if (level > 10)
        level = 10;

    uint8_t duty;

    if (level == 0)
    {
        duty = 0;
    }
    else
    {
        // level 1~10 → duty 63~100 매핑
        // 1 → 63, 10 → 100
        duty = 63 + ((level - 1) * 37 + 4) / 9;
        // 1 -> 63
        // 2 -> 67
        // 3 -> 71
        // 4 -> 75
        // 5 -> 79
        // 6 -> 84
        // 7 -> 88
        // 8 -> 92
        // 9 -> 96
        // 10 -> 100
    }

    uint32_t period  = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t compare = (period + 1) * duty / 100;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare);
}

