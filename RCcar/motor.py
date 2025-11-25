from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor
from time import sleep

mh = Raspi_MotorHAT(addr=0x6f) 
myMotor = mh.getMotor(2) #핀번호

myMotor.setSpeed(100) #속도

try:
	myMotor.run(Raspi_MotorHAT.FORWARD) #배선에 따라 전진 or 후진
	sleep(5)
	myMotor.run(Raspi_MotorHAT.RELEASE) #정지
finally:
	myMotor.run(Raspi_MotorHAT.RELEASE)