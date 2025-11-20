int _read(int file, char *ptr, int len){
	HAL_UART_Receive(&huart2, (uint8_t*)ptr, 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, 1, 100);
	return 1;
}

int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
	return len;
}

void blink_led1() {
  printf("[LED1 Toggle]\r\n");
  HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
}

void blink_led2() {
  printf("[LED2 Toggle]\r\n");
  HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
}

void blink_led3() {
  printf("[LED3 Toggle]\r\n");
  HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
}

unsigned long cnt;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  cnt++;
  if( cnt % 5 == 0 ){
      printf("[LED1 Toggle]\r\n");
      HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
  }
  if( cnt % 3 == 0 ){
      printf("[LED2 Toggle]\r\n");
      HAL_GPIO_TogglePin(GPIOC, LED2_Pin);
  }
  if( cnt % 7 == 0 ){
      printf("[LED3 Toggle]\r\n");
      HAL_GPIO_TogglePin(GPIOC, LED3_Pin);
  }
  printf("[%lu]\r\n", cnt);
}

/*
// gpio.c
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOC, LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}
*/