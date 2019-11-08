from machine import Pin
from time import sleep

def main():
    pin1 = Pin(14, Pin.OUT)
    pin1.on()
    sleep(.005)
    pin1.off()


main()
