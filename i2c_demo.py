from machine import Pin, RTC, TouchPad, Timer, ADC, PWM, I2C
import machine
import network
from ntptime import settime
# import usocket as socket
import urequests as requests
from time import sleep
import esp32
import math
# import ussl

"""
    Pin  13: GPIO #13 : Board LED (Red)
    Pin  14: GPIO #14 : Green LED
    Pin  32: GPIO #32 : Yellow LED
    Pin  15: GPIO #15 : Red LED
    Pin A00: GPIO #26 : Right button switch2
    Pin A01: GPIO #25 : Left button switch1
    Pin A02: GPI  #34 : Potentiometer 1
    Pin A05: GPIO #04 : TouchPad 1 (Bottom)
    Pin A11: GPIO #12 : TouchPad 2 (Top)
"""

# Globals
WIFI_ESSID = 'Verizon-SM-N920V-4E5E'
WIFI_PASSWORD = 'jttt745)'
# WIFI_ESSID = "9HB4U0U"
# WIFI_PASSWORD = "17A729(y"

CLIENT_ID = 'SPINNER1!!!!!!!!'
SERVER = 'farmer.cloudmqtt.com'
PORT = 14167
USER = 'grxrmaim'
PASSWD = '99leuSu_bJd3'

SESSION_ID = b''
ACK = False

POST_ADDR = 'maker.ifttt.com'
POST_URL = \
    'https://maker.ifttt.com/trigger/spinner1_upload/with/key/dAYUFBi1Op3bFw5twpVK29'

# POST_URL = 'https://maker.ifttt.com/trigger/spinner1_upload/with/key/dAYUFBi1Op3bFw5twpVK29'

# press_low_to_high1 = 0
press_high_to_low1 = -1
# press_low_to_high2 = 0
press_high_to_low2 = -1
press_count1 = 0

ax = 0
ay = 0
az = 0

ACCEL_SCALER = 0.00390625

def debounce_low_to_high(button):
    active_time = 0
    while active_time < 100:
        if button.value() == 1:
            active_time += 1
        else:
            return False
        sleep(0.0001)
    return True


def debounce_high_to_low(button):
    active_time = 0
    while active_time < 100:
        if button.value() == 0:
            active_time += 1
        else:
            return False
        sleep(0.001)
    return True


def high_to_low_1_callback(button):
    global press_high_to_low1
    if debounce_high_to_low(button) is True:
        press_high_to_low1 += 1
    return


def low_to_high_1_callback(button):
    global press_low_to_high1
    if debounce_low_to_high(button) is True:
        press_low_to_high1 += 1
    return


def high_to_low_2_callback(button):
    global press_high_to_low2
    if debounce_high_to_low(button) is True:
        press_high_to_low2 += 1
    return


def low_to_high_2_callback(button):
    global press_low_to_high2
    if debounce_low_to_high(button) is True:
        press_low_to_high2 += 1
    return


def check_neg(num, num_bits):
    if (num & (1 << (num_bits - 1))) != 0:
        num = num - (1 << num_bits)
    return num


def tohex(num, num_bits):
    return hex((num + (1 << num_bits)) % (1 << num_bits))


def read_accel_int(i2c_0, accel_i2c_id):
    accel = bytearray(6)
    i2c_0.readfrom_mem_into(accel_i2c_id, 0x32, accel, addrsize=8)
    x = check_neg(int.from_bytes(accel[0:2], 'little', True), 16)
    y = check_neg(int.from_bytes(accel[2:4], 'little', True), 16)
    z = check_neg(int.from_bytes(accel[4:], 'little', True), 16)
    return x, y, z


def avg_accel(i2c_0, accel_i2c_id):
    x_sum = 0
    y_sum = 0
    z_sum = 0
    cnt = 50
    accel = bytearray(6)
    for i in range(cnt):
        i2c_0.readfrom_mem_into(accel_i2c_id, 0x32, accel, addrsize=8)
        #print("Z bytes", accel[4:])
        x_sum += check_neg(int.from_bytes(accel[0:2], 'little', True), 16)
        y_sum += check_neg(int.from_bytes(accel[2:4], 'little', True), 16)
        z_sum += check_neg(int.from_bytes(accel[4:], 'little', True), 16)
    x_avg = x_sum / cnt
    y_avg = y_sum / cnt
    z_avg = z_sum / cnt
    return x_avg, y_avg, z_avg


def raw_to_actual(raw_number):
    g = float(raw_number) * round(ACCEL_SCALER, 7)
    return g


def init_accel(i2c_0, accel_i2c_id):
    if accel_i2c_id == 0:
        print("ERROR: no accel_i2c_id connected")
    else:
        print("I2C_0 connecetd on:", accel_i2c_id)  # 83 = Accel, 72 = Temp
        # Initialize accelerometer
        print("Device ID:", i2c_0.readfrom_mem(accel_i2c_id, 0x00, 1))
        i2c_0.writeto_mem(accel_i2c_id, 0x31, b'\x00')  # Set Resolution
        i2c_0.writeto_mem(accel_i2c_id, 0x2c, b'\x0e')  # Set ODR
        # Enable measurement mode
        i2c_0.writeto_mem(accel_i2c_id, 0x2d, b'\x08')
        # Clear offset registers
        i2c_0.writeto_mem(accel_i2c_id, 0x1e, bytearray([0]))
        i2c_0.writeto_mem(accel_i2c_id, 0x1f, bytearray([0]))
        i2c_0.writeto_mem(accel_i2c_id, 0x20, bytearray([0]))
        print("Accel Initialization completed")

        # Calibrate
        x_avg, y_avg, z_avg = avg_accel(i2c_0, accel_i2c_id)
        x_off = -1 * round(x_avg / 4)
        y_off = -1 * round(y_avg / 4)
        z_off = -1 * round(z_avg / 4)
        print("x_avg:", x_avg, "x_off:", bytearray([x_off]))
        print("y_avg:", y_avg,  "y_off:", bytearray([y_off]))
        print("z_avg:", z_avg,  "z_off:", bytearray([z_off]))
        i2c_0.writeto_mem(accel_i2c_id, 0x1e, bytearray([x_off]))
        i2c_0.writeto_mem(accel_i2c_id, 0x1f, bytearray([y_off]))
        i2c_0.writeto_mem(accel_i2c_id, 0x20, bytearray([z_off]))
        print("Calibrating ...")
        x_avg, y_avg, z_avg = avg_accel(i2c_0, accel_i2c_id)
        print("x_avg:", raw_to_actual(x_avg), "y_avg:",
              raw_to_actual(y_avg), "z_avg:", raw_to_actual(z_avg)-0.85)
        print("Accel Calibration completed")
        print()
    return


def init_temp(i2c_0, temp_i2c_id):
    if temp_i2c_id == 0:
        print("ERROR: no accel_i2c_id connected")
    else:
        print("I2C_0 connecetd on:", temp_i2c_id)
        # Initialize accelerometer
        print("Device ID:", i2c_0.readfrom_mem(temp_i2c_id, 0x0b, 1))
        i2c_0.writeto_mem(temp_i2c_id, 0x03, b'\x80')  # Set Resolution
        print("Temp Sensor Initialization completed")
    return


def detect_temp(i2c_0, temp_i2c_id):
    temp_barr = bytearray(2)
    i2c_0.readfrom_mem_into(temp_i2c_id, 0x00, temp_barr, addrsize=8)
    temp_flt = int.from_bytes(temp_barr, 'big', False)/128
    # print("Temp:", temp, int.from_bytes(temp_barr, 'big', False)/128)
    return temp_flt


"""
START OF PROGRAM
********************************************************************************
"""

# Initialize Buttons
button1_right = Pin(26, Pin.IN, Pin.PULL_DOWN)
button1_right.irq(trigger=Pin.IRQ_FALLING,
                  handler=high_to_low_1_callback)
button2_left = Pin(25, Pin.IN, Pin.PULL_DOWN)
button2_left.irq(trigger=Pin.IRQ_FALLING,
                 handler=high_to_low_2_callback)


# Initialize I2C
i2c_0 = I2C(-1, scl=Pin(22), sda=Pin(23), freq=400000)
temp_i2c_id, accel_i2c_id = i2c_0.scan()


def sessionID_callback(topic, msg):
    global SESSION_ID
    global ACK
    if topic == b"SessionID":
        SESSION_ID = msg
        print((topic, msg))
        print()
    elif topic == b"Acknowledgement":
        ACK = True
        print(msg)
        print()
    else:
        print()
        print("topic didn't match...")
        print()


def wlan_connect(essid, passwd):
    wlan = network.WLAN(network.STA_IF)  # create station interface
    wlan.active(True)  # activate the interface
    if not wlan.isconnected():  # check if the station is connected to an AP
        print("Initiating Wi-Fi connection:")
        wlan.connect(essid, passwd)  # connect to an AP
        print("Connecting to", essid, "...", end="")
        while not wlan.isconnected():
            print(".", end="")
            sleep(0.5)
        print()
    print("Connected to", essid)


def calibrate_sensors(i2c_0, temp_i2c_id, accel_i2c_id):
    init_accel(i2c_0, accel_i2c_id)
    init_temp(i2c_0, temp_i2c_id)


def main():
    global ACK
    global press_high_to_low1
    global press_high_to_low2
    # global press_low_to_high1
    # global press_low_to_high2

    i2c_0 = I2C(-1, scl=Pin(22), sda=Pin(23), freq=400000)
    temp_i2c_id, accel_i2c_id = i2c_0.scan()

    while True:
        temp = detect_temp(i2c_0, temp_i2c_id)
        accel_x, accel_y, accel_z = read_accel_int(i2c_0, accel_i2c_id)

        print("Acceleration x: {}".format(accel_x))
        print("Acceleration y: {}".format(accel_y))
        print("Acceleration z: {}".format(accel_z))
        print("Temperature: {}".format(temp))

        sleep(2)


main()
