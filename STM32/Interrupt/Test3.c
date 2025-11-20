#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>

//MPU6050 의 slave address 7bit 주소이므로, 1비트 쉬프트해서 사용한다.
#define MPU6050 0x68<<1
//가속도 값을 가져올 레지스터 주소 값
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define WHO_AM_I 0x75
//MPU6050 Clock 설정을 위한 레지스터 주소
#define PWR_MGMT_1 0x6B

//가속도 x,y,z 값을 담을 16bit 변수
//i2c 통신을 통해 변경되므로 컴파일러의 방해를 위한 volatile
volatile int16_t ax, ay, az;

void SystemClock_Config(void);


int _read(int file, char *ptr, int len){
	HAL_UART_Receive(&huart2, (uint8_t*)ptr, 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, 1, 100);
	return 1;
}

int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
	return len;
}

void writeI2C(uint8_t addr, uint8_t val, uint8_t size) {
    //레지스터 주소(1바이트) + 데이터(size 바이트)
    uint8_t buf[size + 1];
    //첫 번째 바이트는 레지스터 주소
    buf[0] = addr;
    //이후 바이트에 데이터를 복사
    memcpy(&buf[1], val, size);

    //size + 1 만큼 전송
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050, &buf, size+1, 10);
}

void readI2C(uint8_t addr, uint8_t buf[], uint8_t size){
    //addr 레지스터 주소 읽겠다고 전송
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050, &addr, 1, 10);

    //buf 초기화
    memset(buf, 0, sizeof(buf));
    //buf 에 size 만큼 읽어서 저장, 통신 끝(false)
    HAL_I2C_Master_Receive(&hi2c1, MPU6050, buf, size, 10);
}


int main(void)
{
  HAL_Init();
  SystemClock_Config();


  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  //data 저장할 buf 초기화
  uint8_t buf[100] = {0};
  //문자열로 출력할 str 초기화
  uint8_t str[100] = {0};

  //내부 oscillator 사용
  //PWR_MGMT_1 주소에 0x0을 1(data 크기) byte 쓰기
  writeI2C(PWR_MGMT_1, 0x0, 1);

  while (1)
  {
    //ACCEL_XOUT_H 레지스터로부터 6 byte 읽어서 buf 에 저장
    readI2C(ACCEL_XOUT_H, buf, 6); // ACCEL 값 읽기
    readI2C(WHO_AM_I, buf, 1); // MPU6050의 Slave Address 출력
    readI2C(GYRO_XOUT_H, buf, 6); // GYRO 값 읽기

    //x,y,z 파싱
    ax = (buf[0] << 8) | buf[1];
    ay = (buf[2] << 8) | buf[3];
    az = (buf[4] << 8) | buf[5];

    //출력
    sprintf(str, "%d, %d, %d\r\n", ax, ay, az);
    printf("%s", str);
    HAL_Delay(50);
  }
}