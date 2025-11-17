from gpiozero import LED
from time import sleep

red1 = LED(14) #GPIO pin number
red2 = LED(15)
red3 = LED(18)

while True:
    num = input("INPUT>> ")
    
    if num == '1':
        red1.toggle()
    elif num == '2':
        red2.toggle()
    elif num == '3':
        red3.toggle()
        
    sleep(0.2)

