from gpiozero import PWMLED
from time import sleep

led = PWMLED(14)

while True:
    num = input("INPUT>> ")

    if num == '1':
        if led.value <= 0:
            led.value = 0
        else:
            led.value -= 0.1
    elif num == '2':
        if led.value >= 1:
            led.value = 1
        else:
            led.value += 0.1
        
    sleep(0.2)

