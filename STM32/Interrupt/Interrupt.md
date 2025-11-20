# Interrupt

## 1. 보드 정보
- **보드 이름**: Nucleo
- **보드 모델명**: Nucleo-F103RB
- **ST사의 CPU 칩셋 이름**: STM32F103RB
- **CPU 설계**: ARM Cortex-M3

<br> 
<br> 

![STM32Arduion](../LED_UART/images/STM32_Arduino_Connector.png)

## [Test 1]
Interrupt 우선순위 테스트  
 
Bread board에 버튼 2개와 LED 2개를 연결  
- 버튼1 누르면, LED1 무한 blink  
- 버튼2 누르면, LED2 무한 blink  

아래와 같은 우선 순위 상황을 만들고 테스트
1) 버튼1 > 버튼2  
2) 버튼1 < 버튼2  
3) 버튼1 == 버튼2  

<img src="./images/image_test1_ioc.png" height="500">

<p align="left">
  <img src="./images/image_test1_1.jpg" height="300">
  <img src="./images/image_test1_2.jpg" height="300">
</p>


## [Test 2]
STM32 Nucleo Board PWM Example  

Delay 함수를 이용하여 LED의 밝기를 조절하기
- 힌트:
    - 50ms 껐다가, 50ms 켰다를 빠르게 반복하면 LED가 어두워진다.
    - 70ms 껐다가, 30ms 켰다를 빠르게 반복하면 LED가 더 어두어진다.

