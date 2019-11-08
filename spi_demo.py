from machine import Pin, SPI
from time import sleep

def main():
    '''
    SPI demo for ece49022 skills assessment

    SCK  => Pin 14
    CS   => Pin 26
    MOSI => Pin 13
    MISO => Pin 12
    '''

    message = b'abc'
    spi = SPI(0)
    chip_select = Pin(26, Pin.OUT, 1)

    spi.init(baudrate=1_000_000, polarity=1, phase=0, bits=8, firstbit=SPI.MSB,
             sck=Pin(14), mosi=Pin(13), miso=Pin(12))

    for char in message:
        chip_select.off()
        spi.write(char)
        chip_select.on()

    sleep(2)


main()
