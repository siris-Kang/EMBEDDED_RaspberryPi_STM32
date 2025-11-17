from gpiozero import Button, PWMLED
from signal import pause
from time import sleep

bright = 0

def press1():
    led.value = min(led.value + 0.1, 0.9)

def press2():
    led.value = max(led.value - 0.1, 0.1)

btn1 = Button(15)
btn2 = Button(18)
led = PWMLED(14)
led.off()

btn1.when_pressed = press1
btn2.when_pressed = press2

pause()