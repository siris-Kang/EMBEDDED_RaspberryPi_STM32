int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();

  while (1)
  {
    /* USER CODE END WHILE */
      // level 1

      HAL_GPIO_WritePin(GPIOC, LED1_Pin, 1);
      HAL_Delay(500);
      HAL_GPIO_WritePin(GPIOC, LED2_Pin, 1);
      HAL_Delay(500);
      HAL_GPIO_WritePin(GPIOC, LED3_Pin, 1);
      HAL_Delay(500);

      HAL_GPIO_WritePin(GPIOC, LED1_Pin, 0);
      HAL_GPIO_WritePin(GPIOC, LED2_Pin, 0);
      HAL_GPIO_WritePin(GPIOC, LED3_Pin, 0);
      HAL_Delay(500);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/*
// gpio.c 
void MX_GPIO_Init(void)
{
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