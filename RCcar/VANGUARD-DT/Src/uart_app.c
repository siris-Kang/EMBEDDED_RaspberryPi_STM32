/*
 * uart_app.c
 *
 *  Created on: Nov 27, 2025
 *      Author: SSAFY
 */


// uart_app.c
#include "uart_app.h"
#include "usart.h"
#include "motor.h"
#include "servo.h"
#include <string.h>

static uint32_t last_cmd_tick = 0;

void Uart_App_Init(void)
{
  char *start_msg = "STM32 RC Car Ready\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)start_msg, strlen(start_msg), 100);

  Motor_SetDirection(MOTOR_DIR_STOP);
  Servo_SetAngle(75);   // MIDDLE
  last_cmd_tick = HAL_GetTick();
}

void Uart_App_Task(void)
{
  uint8_t ch;

  // 1) 명령 수신 처리
  if (HAL_UART_Receive(&huart2, &ch, 1, 10) == HAL_OK) {

    // 확인용 에코
    HAL_UART_Transmit(&huart2, &ch, 1, 10);

    switch (ch) {
      case 'w':
        Motor_SetDirection(MOTOR_DIR_FORWARD);
        last_cmd_tick = HAL_GetTick();
        break;
      case 's':
        Motor_SetDirection(MOTOR_DIR_BACKWARD);
        last_cmd_tick = HAL_GetTick();
        break;
      case 'x':
        Motor_SetDirection(MOTOR_DIR_STOP);
        last_cmd_tick = HAL_GetTick();
        break;
      case 'a':
        Servo_SetAngle(0);
        last_cmd_tick = HAL_GetTick();
        break;
      case 'd':

	Servo_SetAngle(150);
        last_cmd_tick = HAL_GetTick();
        break;
      case 'c':
        Servo_SetAngle(75);
        last_cmd_tick = HAL_GetTick();
        break;
      default:
        break;
    }
  }

  // 2) 명령 없으면 자동 정지
  uint32_t now = HAL_GetTick();
  if (now - last_cmd_tick > 500) {
    Motor_SetDirection(MOTOR_DIR_STOP);
    Servo_SetAngle(75);
    // last_cmd_tick은 업데이트하지 않아서 이 상태 유지
  }
}

