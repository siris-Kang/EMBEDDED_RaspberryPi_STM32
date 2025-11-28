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

// 마지막 명령 시간 (원래 있던 전역이라고 가정)
extern uint32_t last_cmd_tick;

void Uart_App_Task(void)
{
    uint8_t ch;
    static uint16_t speed_buf = 0;   // 현재까지 입력된 속도 (0~100)
    static uint8_t  speed_len = 0;   // 몇 자리 입력했는지

    // 1바이트 수신 (없으면 그냥 리턴)
    if (HAL_UART_Receive(&huart2, &ch, 1, 10) != HAL_OK) {
        return;
    }

    // 1) 숫자면: 속도 누적
    if (ch >= '0' && ch <= '9')
    {
        if (speed_len < 3) {   // 최대 세 자리만 허용
            speed_buf = speed_buf * 10 + (ch - '0');
            if (speed_buf > 100)
                speed_buf = 100;
            speed_len++;

            uint8_t duty = (uint8_t)speed_buf;
            Motor_SetSpeed(duty);

            Uart_Print("\r\n[UART] speed_buf=%u, duty=%u%%\r\n", speed_buf, duty);
            last_cmd_tick = HAL_GetTick();
        }
        return;   // 숫자인 경우, 여기서 끝
    }

    // 2) 숫자가 아닌 문자가 들어오면, 속도 입력은 끝난 걸로 보고 버퍼 초기화
    speed_buf = 0;
    speed_len = 0;

    // 개행 문자는 그냥 무시
    if (ch == '\r' || ch == '\n') {
        return;
    }

    // 3) 문자 명령 처리
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

    case 'h':  // 핸드셰이크 테스트용
        Uart_Print("\r\n[MCU] HELLO FROM STM32\r\n");
        break;

    default:
        Uart_Print("\r\n[CMD] unknown '%c'\r\n", ch);
        break;
    }
}
