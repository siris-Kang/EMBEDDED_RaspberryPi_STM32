import sys
sys.path.append("../motor")
import RPi.GPIO as gpio
import time
from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor

# set ultrasonic sensor
TRIGGER = 24
ECHO = 23

gpio.setmode(gpio.BCM)
gpio.setup(TRIGGER, gpio.OUT)
gpio.setup(ECHO, gpio.IN)

# set dc motor
mh = Raspi_MotorHAT(addr=0x6f)
myMotor = mh.getMotor(2)
myMotor.setSpeed(130)

def get_distance():
    """초음파 센서로 거리(cm) 측정해 반환"""
    gpio.output(TRIGGER, gpio.LOW)
    time.sleep(0.0002)

    # 트리거 HIGH
    gpio.output(TRIGGER, gpio.HIGH)
    time.sleep(0.00001)  # 10us
    gpio.output(TRIGGER, gpio.LOW)

    # Echo LOW 대기
    while gpio.input(ECHO) == gpio.LOW:
        start = time.time()

    # Echo HIGH 대기
    while gpio.input(ECHO) == gpio.HIGH:
        end = time.time()

    # 거리 계산
    duration = end - start
    distance = (duration * 34300) / 2
    return distance


try:
    print("Running... press Ctrl+C to stop")

    while True:
        dist = get_distance()
        print("Distance:", round(dist, 2), "cm")

        # 10cm 이하 → 멈춤
        if dist < 10:
            print("STOP")
            myMotor.run(Raspi_MotorHAT.RELEASE)
        else:
            print("GO")
            myMotor.run(Raspi_MotorHAT.FORWARD)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped")

finally:
    myMotor.run(Raspi_MotorHAT.RELEASE)
    gpio.cleanup()
