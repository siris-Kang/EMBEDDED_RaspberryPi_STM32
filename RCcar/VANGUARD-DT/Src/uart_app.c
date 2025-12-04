/*
 * uart_app.c
 *
 *  Created on: Nov 27, 2025
 *      Author: SSAFY
 */


#include "uart_app.h"
#include "motor.h"
#include "servo.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart2;

static uint32_t last_cmd_tick = 0;

void Uart_App_Init(void)
{
  char *start_msg = "STM32 RC Car Ready\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)start_msg, strlen(start_msg), 100);

  Motor_SetDirection(MOTOR_STOP);
  Servo_SetAngle(75);   // MIDDLE
  last_cmd_tick = HAL_GetTick();
}


void Uart_Print(const char *fmt, ...)
{
    char buf[64];

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, 100);
    }
}

extern uint32_t last_cmd_tick;

void Uart_App_Task(void)
{
    uint8_t ch;

    // 수신
    if (HAL_UART_Receive(&huart2, &ch, 1, 10) != HAL_OK) {
        return;
    }

    // Level 속도
    if (ch >= '0' && ch <= '9')
    {
        uint8_t level = (uint8_t)(ch - '0');

        Motor_SetSpeed(level);

        Uart_Print("\r\n[UART] level=%u\r\n", level);
        last_cmd_tick = HAL_GetTick();
        return;
    }
    if (ch == '\r' || ch == '\n') {
        return;
    }

    switch (ch)
    {
    case 'w':
        Motor_SetDirection(MOTOR_FORWARD);
        Uart_Print("\r\n[CMD] FORWARD\r\n");
        last_cmd_tick = HAL_GetTick();
        break;

    case 's':
        Motor_SetDirection(MOTOR_BACKWARD);
        Uart_Print("\r\n[CMD] BACKWARD\r\n");
        last_cmd_tick = HAL_GetTick();
        break;

    case 'x':
        Motor_SetDirection(MOTOR_STOP);
        Uart_Print("\r\n[CMD] STOP\r\n");
        last_cmd_tick = HAL_GetTick();
        break;

    case 'a':
        Servo_SetAngle(0);
        Uart_Print("\r\n[CMD] LEFT: SERVO 0deg\r\n");
        last_cmd_tick = HAL_GetTick();
        break;

    case 'd':
        Servo_SetAngle(150);
        Uart_Print("\r\n[CMD] RIGHT: SERVO 150deg\r\n");
        last_cmd_tick = HAL_GetTick();
        break;

    case 'c':
        Servo_SetAngle(75);
        Uart_Print("\r\n[CMD] CENTER: SERVO 75deg\r\n");
        last_cmd_tick = HAL_GetTick();
        break;

    case 'h':
        Uart_Print("\r\n[MCU] HELLO FROM STM32\r\n");
        break;

    default:
        Uart_Print("\r\n[CMD] unknown '%c'\r\n", ch);
        break;
    }
}

