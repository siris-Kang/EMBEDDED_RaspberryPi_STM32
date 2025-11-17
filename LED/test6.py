from gpiozero import Button, LED
from time import sleep

mode = 0

def press():
    print("Change Mode!")
    global mode
    mode = 1 - mode

ledRed = LED(15)
ledGreen = LED(18)
btn = Button(14)

btn.when_pressed = press

while True:
    if mode == 0:
        ledRed.on()
        sleep(0.5)
        ledRed.off()
        ledGreen.on()
        sleep(0.5)
        ledGreen.off()
        
    else:
        ledRed.on()
        ledGreen.on()
        sleep(0.5)
        ledRed.off()
        ledGreen.off()
        sleep(0.5)
