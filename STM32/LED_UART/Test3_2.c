int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  int mode = 0;

  while (1)
  {
      int btnstate1 = HAL_GPIO_ReadPin(GPIOA, BTN1_Pin);
      int btnstate2 = HAL_GPIO_ReadPin(GPIOA, BTN2_Pin);
      if ( btnstate1 == 0 ){
        HAL_Delay(50);
        mode = 0;
        HAL_GPIO_WritePin(GPIOC, LED1_Pin, 0);
        HAL_GPIO_WritePin(GPIOC, LED2_Pin, 0);
        HAL_GPIO_WritePin(GPIOC, LED3_Pin, 0);
      }
      if (btnstate2 == 0){
        HAL_Delay(50);
        mode = 1;
        HAL_GPIO_WritePin(GPIOC, LED1_Pin, 0);
        HAL_GPIO_WritePin(GPIOC, LED2_Pin, 0);
        HAL_GPIO_WritePin(GPIOC, LED3_Pin, 0);
      }

      if (mode == 0) {
        HAL_GPIO_WritePin(GPIOC, LED1_Pin, 1);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOC, LED1_Pin, 0);
        HAL_Delay(200);

        HAL_GPIO_WritePin(GPIOC, LED2_Pin, 1);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOC, LED2_Pin, 0);
        HAL_Delay(200);

        HAL_GPIO_WritePin(GPIOC, LED3_Pin, 1);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOC, LED3_Pin, 0);
        HAL_Delay(200);
      }
      else if (mode == 1) {
        HAL_GPIO_WritePin(GPIOC, LED3_Pin, 1);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOC, LED3_Pin, 0);
        HAL_Delay(200);

        HAL_GPIO_WritePin(GPIOC, LED2_Pin, 1);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOC, LED2_Pin, 0);
        HAL_Delay(200);

        HAL_GPIO_WritePin(GPIOC, LED1_Pin, 1);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOC, LED1_Pin, 0);
        HAL_Delay(200);
      }
  }
}