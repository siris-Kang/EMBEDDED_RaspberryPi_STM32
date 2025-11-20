## LED & UART

## 1. 보드 정보
- **보드 이름**: Nucleo
- **보드 모델명**: Nucleo-F103RB
- **ST사의 CPU 칩셋 이름**: STM32F103RB
- **CPU 설계**: ARM Cortex-M3

<br> 
<br> 

![STM32Arduion](./images/STM32_Arduino_Connector.png)

### [Test 1]
LED 깜빡이기  
 
Bread Board에 LED 3개를 연결하여 **PWM**을 이용해 밝기 조절하기  
- 회로 연결:
    - 각 LED는 **220옴 저항**과 함께 연결되어야 합니다.
    - LED 1, 2, 3을 **동시에 깜빡이기**.
    - LED 1만 켜기 → LED 2만 켜기 → LED 3만 켜기 → LED 1만 켜기 (반복)  

![test1](./images/image_test1.png)

### [Test 2]
STM32 Nucleo Board PWM Example  

Delay 함수를 이용하여 LED의 밝기를 조절하기
- 힌트:
    - 50ms 껐다가, 50ms 켰다를 빠르게 반복하면 LED가 어두워진다.
    - 70ms 껐다가, 30ms 켰다를 빠르게 반복하면 LED가 더 어두어진다.

### [Test 3_1]
Bread Board에 버튼 2개 연결하기  
버튼1 누르면 LED 3개 ON, 다시 누르면 LED 3개 OFF  

![chip_set](./images/image_chip.png)

### [Test 3_2]
Bread Board에 버튼 2개 연결하기  
LED는 계속 깜빡인다.

- 버튼1 누르면 LED1 → LED2 → LED3 → LED1 → …  
- 버튼2 누르면 LED3 → LED2 → LED1 → LED3 → …  


![test3_2](./images/image_test3_2.jpg)


### UART 설정
USART vs UART

- USART : Sync(동기) 통신
    - Clock PIN 을 하나 더 사용하여, Clock을 기준으로 Sync(박자)를 맞춘다.  
    - PIN 3개 사용 (TX, RX, CK)  
- UART : Async(비동기) 통신  
    - Clock PIN 없이, Start / End bit를 추가적으로 사용하여 통신  
    - PIN 2개 사용 (TX, RX)  

- 설정  
Categories → Connectivity → USART2  
Asynchronous로 체크  

<br>

**printf/scanf**  
main 위에 붙여넣기
```
int _read(int file, char *ptr, int len){
	HAL_UART_Receive(&huart2, (uint8_t*)ptr, 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, 1, 100);
	return 1;
}

int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
	return len;
}
```