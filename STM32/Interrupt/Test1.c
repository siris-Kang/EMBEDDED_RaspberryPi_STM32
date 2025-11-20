int btn1_mode = 0;
int btn2_mode = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if( GPIO_Pin == BTN1_Pin){
      btn1_mode = 1;
  }
  if( GPIO_Pin == BTN2_Pin){
      btn2_mode = 1;
  }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  while (1)
  {
      if (btn1_mode == 1) {
	  HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
	  HAL_Delay(100);
      }
      if (btn2_mode == 1) {
      	  HAL_GPIO_TogglePin(GPIOC, LED2_Pin);
      	  HAL_Delay(100);
      }
  }
}




/*
// gpio.c
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOC, LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


// stm32f1xx_it.c <- it에는 interrupt 관련 API들이 있음
void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(BTN2_Pin);
}

void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(BTN1_Pin);
}


//stm32f1xx_hal_gpio.c <- HAL_GPIO_EXTI_IRQHandler를 눌러서 넘어갈 수 있음
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin)
{
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_Pin) != 0x00u)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    HAL_GPIO_EXTI_Callback(GPIO_Pin);
  }
}

__weak void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // weak이니 main에서 오버로딩 가능 -> main으로 가져오기
{
  UNUSED(GPIO_Pin);
}
*/