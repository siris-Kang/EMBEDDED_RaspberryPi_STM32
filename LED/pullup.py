from gpiozero import Button
from signal import pause

def press():
    print("Btn Pressed!")

def release():
    print("Btn Released!")

btn = Button(18)

btn.when_pressed = press
btn.when_released = release

pause()