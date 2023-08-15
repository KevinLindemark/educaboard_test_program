from machine import UART, Pin
from time import ticks_ms

def test_gps_pps():
    #########################################################################
    # CONFIGURATION
    gpsPort = 2                                 # ESP32 UART port
    gpsSpeed = 9600                             # UART speed

    #########################################################################
    # OBJECTS

    uart = UART(gpsPort, gpsSpeed)              # UART object creation

    ba = bytearray(42)
    ba[0] = 0xB5
    ba[1] = 0x62
    ba[2] = 0x06
    ba[3] = 0x31
    ba[4] = 0x20
    ba[5] = 0x00
    ba[6] = 0x00
    ba[7] = 0x01
    ba[8] = 0x00
    ba[9] = 0x00
    ba[10] = 0x32
    ba[11] = 0x00
    ba[12] = 0x00
    ba[13] = 0x00
    ba[14] = 0x05
    ba[15] = 0x00
    ba[16] = 0x00
    ba[17] = 0x00
    ba[18] = 0x05
    ba[19] = 0x00
    ba[20] = 0x00
    ba[21] = 0x00
    ba[22] = 0x00
    ba[23] = 0x00
    ba[24] = 0x00
    ba[25] = 0x80
    ba[26] = 0x00
    ba[27] = 0x00
    ba[28] = 0x00
    ba[29] = 0x80
    ba[30] = 0x00
    ba[31] = 0x00
    ba[32] = 0x00
    ba[33] = 0x00
    ba[34] = 0xEF
    ba[35] = 0x00
    ba[36] = 0x00
    ba[37] = 0x00
    ba[38] = 0x83
    ba[39] = 0xFA
    ba[40] = 0x0D
    ba[41] = 0x0A

    print(uart.write(ba, 42))
    pps_pin = Pin(5, Pin.IN)
    led = Pin(26, Pin.OUT)
    start = ticks_ms()
    testing = True
    while testing:
        led.value(pps_pin.value())
        if ticks_ms() - start > 10000:
            print("Stopped")
            testing = False
            break
test_gps_pps()