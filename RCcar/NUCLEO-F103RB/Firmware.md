# NUCLEO-F103RB Setting

## DC Motor


## Servo Motor


## UART
UART에서 패킷을 설정하여 통신을 원활하게 하였다.  
라즈베리파이에서 UART로 데이터를 보내고 모터를 구동시킨다.  

### 패킷 설정
EX) PKT: AA 55 01 14 F6 01 6B  

[0] header1 = 0xAA  
[1] header2 = 0x55  
[2] seq     = 0~255  
[3] speed   = int8_t (-100 ~ 100)  
[4] steer   = int8_t (-100 ~ 100)  
[5] flags   = uint8_t (bit0: enable, bit1: emergency stop)  
[6] checksum = (byte0 + ... + byte5) & 0xFF  

0-1) 0xAA = 1010 1010
0x55 = 0101 0101  
서로 비트가 완전히 반대라서 랜덤 노이즈가 우연히 만들어낼 확률이 낮기에 동기화(synchronization)를 맞추기 위해 사용

2) seq: 라즈베리파이가 패킷을 보낼 때마다 seq를 0,1,2,3… 이런 식으로 증가시키고 STM32는 g_cmd.seq에 이 값을 저장

5) flags: 확장성 용도

6) 오류가 생긴 패킷 무시 용도

### Build
```
g++ -std=c++17 main.cpp uart.cpp -o uart
```

