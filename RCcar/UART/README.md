# Raspberry-pi
라즈베리파이와 SMT보드의 통신을 위해,  
라즈베리파이에서 UART 프로그램을 작성 후 빌드한다.  

```
./UART/
      ├─ main.cpp
      ├─ uart.cpp
      └─ uart.hpp
```

단, main.cpp의 `dev = "/dev/ttyACM0"` 의 시리얼 포트를 꼭 수정하도록 한다.  

### Build
```
g++ -std=c++17 main.cpp uart.cpp -o uart
```

### 실행
```
./uart
```
프로그램을 실행하면 라즈베리파이에서 STM32 보드로 패킷을 전송하고 받을 수 있다.  
`Ex) 20 0 1`
- speed (uint8_t)
- steer (uint8_t)
- flags (uint8_t, bit0: enable, bit1: emergency stop)  


STM32 보드는 수신한 패킷을 그대로 재전송(echo)하여, 패킷이 정상적으로 전송·수신되고 있는지 확인할 수 있다.  

logs/YYMMDD_HHMM.txt 형식의 파일로  
터미널에 출력되는 STM32 로그가 그대로 저장된다.  