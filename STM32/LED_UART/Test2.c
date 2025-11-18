#include "main.h"
#include "gpio.h"

void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();

  uint8_t up = 1;
  uint8_t onTime = 10;
  uint8_t offTime;

  /* Infinite loop */
  while (1)
  {
      offTime = 100 - onTime;

      HAL_GPIO_WritePin(GPIOC, LED1_Pin, 1);  // LED 켬
      HAL_Delay(onTime);

      HAL_GPIO_WritePin(GPIOC, LED1_Pin, 0);  // LED 끔
      HAL_Delay(offTime);

      if (up) {
        if (onTime + 5 > 100) {
            up = 0;
            onTime = 95;
        }
        else {
            onTime += 5;
        }
      }
      else {
        if (onTime - 5 < 0) {
            up = 1;
            onTime = 5;
        }
        else {
            onTime -= 5;
        }
  }
}

/*
// gpio.c -> To get PIN information
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}
*/