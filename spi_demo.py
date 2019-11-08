from machine import Pin, SPI
from time import sleep

def main():
    '''
    SPI demo for ece49022 skills assessment

    SCK  => Pin 25 => A1
    CS   => Pin 26 => A0
    MOSI => Pin 13 => 13
    MISO => Pin 12 => 12
    '''

    message = ['a', 'b', 'c']

    spi = SPI(-1, sck=Pin(25), mosi=Pin(13), miso=Pin(12), polarity=1, phase=0,
              bits=8, firstbit=SPI.MSB)

    chip_select = Pin(26, Pin.OUT, 1)

    # baud rate, polarity, phase, bits, firstbit
    spi.init(baudrate=1_000_000)

    sleep(1)

    for char in message:
        print(char)
        chip_select.off()
        spi.write(bytes(char, 'utf-8'))
        chip_select.on()

    sleep(5)


main()
