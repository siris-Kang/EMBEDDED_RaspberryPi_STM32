/*
 * servo.c
 *
 *  Created on: Nov 27, 2025
 *      Author: SSAFY
 */


#include "servo.h"
#include "tim.h"

// TIM3_CH1, PA6에 PWM 설정

void Servo_Init(void)
{
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void Servo_SetAngle(uint8_t angle)
{
  if (angle > 180) angle = 180;

  // 타이머: 1us 해상도, Period = 19999 (20ms)
  // 1ms ~ 2ms 펄스 → 0~180도
  const uint16_t min_pulse = 1000;   // 1.0ms
  const uint16_t max_pulse = 2000;   // 2.0ms

  uint16_t pulse = min_pulse + (uint16_t)((max_pulse - min_pulse) * angle / 180);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
}
