from machine import Pin, ADC, UART, I2C
from time import sleep, ticks_ms
from gps_GPGGA_GPZDA import GPS_GPGGA_GPZDA
from rotary_encoder import rotary_encoder_tester
from gpio_lcd import GpioLcd
from eeprom_navn import eeprom_student_navn
from port_expander_led_23 import led_og_port_exp_tester, kill
import _thread

class opsummering:
    led = ""
    potentiometer = ""
    knapper = ""
    LMT84 = ""
    gps = ""
    gps_pps = ""
    rotary_encoder = ""
    lcd = ""
    EEPROM = ""
    port_exp = ""
    def status():
        print("\n*** Opsummering af testen: ***\n")
        print(f"\n{opsummering.led}\n{opsummering.potentiometer}\n{opsummering.knapper}\n{opsummering.LMT84}\n{opsummering.gps}\n{opsummering.gps_pps}\n{opsummering.rotary_encoder}\n{opsummering.lcd}\n{opsummering.EEPROM}\n{opsummering.port_exp}")
        print("\n*** HUSK AT GEMME OPSUMMERINGEN TIL UNDERVISEREN HVIS DU SKAL HAVE HJÆLP ***")
        print("*** HUSK AT SKRIVE NAVN PÅ DIT EDUCABOARD ***")

def test_gps_pps():
    print("Tester GPS PPS i 10 sekunder, kontrollér om både den grønne LED på GPS modulet og LED1 blinker")
    #########################################################################
    # CONFIGURATION
    gpsPort = 2                                 # ESP32 UART port
    gpsSpeed = 9600                             # UART speed

    #########################################################################
    # OBJECTS

    uart = UART(gpsPort, gpsSpeed)              # UART object creation
    ba = bytearray([
        0xB5, 0x62,             # Header
        0x06, 0x31,             # ID 
        0x20, 0x00,             # Fixed length payload 32 bytes
        0x00,                   # 0, tpIdx
        0x01,                   # 1, reserved0
        0x00, 0x00,             # 2, reserved1
        0x32, 0x00,             # 4, antCableDelay
        0x00, 0x00,             # 6, rfGroupDelay
        0x05, 0x00, 0x00, 0x00, # 8, freqPeriod
        0x05, 0x00, 0x00, 0x00, # 12, freqPeriodLock
        0x00, 0x00, 0x00, 0x80, # 16, pulseLenRatio
        0x00, 0x00, 0x00, 0x80, # 20, pulseLenRatioLock
        0x00, 0x00, 0x00, 0x00, # 24, userConfigDelay
        0xEF, 0x00, 0x00, 0x00, # 28, Flags
        0x83, 0xFA,             # Checksum
        0x0D, 0x0A ])           # <CR><LF>

    uart.write(ba, 42)
    pps_pin = Pin(5, Pin.IN)
    led = Pin(26, Pin.OUT)
    start = ticks_ms()
    testing = True
    opsummering.gps_pps = ""
    while testing:
        led.value(pps_pin.value())
        if ticks_ms() - start > 10000:
            pps_gps_svar = input("Blinkede både den grønne LED på GPS modulet og LED1 i 5Hz? ja/nej/forfra\n").lower()
            if pps_gps_svar == "ja":
                print("👍 GPS PPS virker")
                return "👍 GPS PPS virker"
            elif pps_gps_svar == "forfra":
                test_gps_pps()
            else:
                print("GPS PPS virker ikke")
                return "GPS PPS virker ikke"

def gps_tester():
    uart = UART(2, 9600)              # UART object creation
    gps = GPS_GPGGA_GPZDA(uart)
    print("Tester GPS")
    count = 0
    while True:      
        if count > 100000:
                print("GPS, virker ikke")
                opsummering.gps = "GPS virker ikke"
                break           
        if (gps.receive_nmea_data(True)):
            print(count)
            gps_frames = gps.get_test_frames()
            if gps_frames[0] == True or gps_frames[1] == True:
                print("👍 GPS virker!")
                opsummering.gps = "👍 GPS virker"
                break
            else:
                print("GPS, virker ikke")
                opsummering.gps = "GPS virker ikke"
                break
        count +=1

def led_tester():
    print("Tester LED'er og Port expander. blinker en grøn, gul og rød LED på educaboardet? (ja/nej)\n")
    port_exp_led_blink_thread = _thread.start_new_thread(led_og_port_exp_tester, ())

    svar_led = input()
    if svar_led.lower() == "ja":
     print("👍 LED'er virker")   
     opsummering.led = "👍 LED'er virker"
     # TODO kill thread after use
     kill["thread1"] = True
     
     print("👍 Port expander virker")   
     opsummering.port_exp = "👍 Port expander virker"
         
    else:
     print("LED'er virker ikke")
     print("Gennemgå LED kredsløbet for fejl og prøv igen.")
     print("Port expander virker ikke")   
     opsummering.port_exp = "Port expander virker ikke"
     kill["thread1"] = True
     sleep(3)
     opsummering.led = "LED'er virker ikke"
     
def potentiometer_tester():
    print("Tester potentiometer i 10 sekunder. Vises værdier mellem 0 og 4095 når der skrues på potentiometeret?")
    sleep(4)
    pot = ADC(Pin(34))
    count = 0
    while count < 100:
        print(pot.read())
        count +=1
        sleep(0.1)
    print("Blev værdier mellem 0 og 4095 vist når der blev skruet på potentiometeret? (ja/nej/forfra)\n")
    svar_potentiometer = input().lower()
    if svar_potentiometer == "ja":
        print("👍 Potentiometeret virker")
        opsummering.potentiometer = "👍 Potentiometeret virker"
    elif svar_potentiometer == "forfra":
      print("Tester potentiometer igen, kig i shell og skru på potentiometeret")
      sleep(3)
      potentiometer_tester()
    else:
      print("Gennemgå LED kredsløbet for fejl og prøv igen. Lukker testprogram")
      opsummering.potentiometer = "Der er problemer med potentiometeret"
      
def knap_tester():
    led1 = Pin(26, Pin.OUT)
    led1.value(0)
    print("Tester tryknapperne i 10 sekunder, når pb1 trykkes bør LED1 lyse, og når PB2 holdes nede bør LED1 blinke")
    pb1 = Pin(4, Pin.IN)                 # External pull-up and debounce
    pb2 = Pin(0, Pin.IN)                 # Direct connection with pull-up thus inverted
    count = 0
    while count < 100:
        val1 = pb1.value()
        val2 = pb2.value()  
        count +=1
        sleep(0.1)
        if val1 == 0:
            led1.value(1)
        elif val2 == 0:
            led1.value(not led1.value())
            sleep(0.1)
        else:
            led1.value(0)
    svar_knap = input("virker tryknapperne? (når PB1 holdes nede bør LED1 lyse, og når PB2 holdes nede bør LED1 blinke) ja/nej/forfra\n")
    if svar_knap == "ja":
      print("👍 Trykknapper virker")
      opsummering.knapper = "👍 Trykknapper virker"
    elif svar_knap == "forfra":
      knap_tester()
    else:
      print("Trykknapper virker ikke")
      opsummering.knapper = "Trykknapper virker ikke"

def afslut():
    afslut = input("test er slut - afslut og se opsummering? ja/nej\n")
    if afslut == "ja":
      opsummering.status()
      global testing
      testing = False
    else:
      print("Kører testen igen")

def i2c_ping_EEPROM():
    # husk at importere I2C fra machine modulet
    print("Tester EEPROM I2C")
    try:
        i2c = I2C(0, freq = 400000)                 # I2C H/W 0 object
        i2c.readfrom(0x50, 0)
        print("👍 EEPROM I2C virker")
        return "👍 EEPROM I2C virker"
    except:
        print("EEPROM I2C virker ikke")
        return "EEPROM I2C virker ikke"

def lmt84_tester():
    # TODO lav en automatisk test uden bruger input (temp mellem range og range mellem 10 - 30)
    print("LMT84 test\n")
    adcVal = 0
    adcLmt84 = ADC(Pin(35))             
    adcLmt84.atten(ADC.ATTN_6DB)
    adc2mV = 2100.0 / 4095.0
    # V = (-5.50 mV/°C) T + 1035 mV
    # T = (V - 1035 mV) / (-5.5 mV/°C)
    alpha = -5.5
    beta = 1035
    average = 16
    if average > 1:
        for i in range (average):
            adcVal += adcLmt84.read()
            sleep(1 / average)
        adcVal = adcVal / average
    else:
        adcVal = adcLmt84.read()
        sleep(1)

    mV = adc2mV * adcVal
    temp = (mV - beta) / alpha
    print(f"ADC:{adcVal} \ntemp: {temp}°C")
    if int(temp) in range(10, 31):
        print("👍 LMT84 temperatursensor virker")
        opsummering.LMT84 = "👍 LMT84 temperatursensor virker"
    #svar_temp = input("Vises nogenlunde korrekt temperatur fra LMT84 sensor? ja/nej\n").lower()
    #if svar_temp == "nej":
    else:
        print("LMT84 temperatursensor virker ikke")
        opsummering.LMT84 = "LMT84 temperatursensor virker ikke"

def lcd_tester():
    print("LCD 20x4 test (Husk at sætte JP5 jumper i position R7/til venstre.)\nkan du se teksten på Educa boardets display? ja/nej?")

    # Create the LCD object
    lcd = GpioLcd(rs_pin=Pin(27), enable_pin=Pin(25),
                  d4_pin=Pin(33), d5_pin=Pin(32), d6_pin=Pin(21), d7_pin=Pin(22),
                  num_lines=4, num_columns=20)
    lcd.clear()
    sleep(2)
    lcd.putstr('* Educaboard ESP32 *')

    lcd.move_to(1, 1)
    lcd.putstr('KEA ITT www.kea.dk')

    lcd.move_to(1, 2)
    lcd.putstr('HD44780 LCD 4 bits')

    lcd.move_to(0, 3)
    # happy_face = bytearray([0x00, 0x0A, 0x00, 0x04, 0x00, 0x11, 0x0E, 0x00])
    # lcd.custom_char(0, happy_face)
    # lcd.putchar(chr(0))
    lcd.putstr('Velkommen til KEA :)')
    svar_lcd = input().lower()
    if svar_lcd == "ja":
        return "👍 lcd display virker"
    else:
        print("justér det blå trimmepotentiometer så du ser tekst eller firkanter")
        return "lcd display virker ikke"

testing = True
if __name__ == "__main__":
    while testing:
        eeprom_student_navn()
        led_tester()
        potentiometer_tester() 
        knap_tester()
        lmt84_tester()
        opsummering.gps_pps = test_gps_pps()
        gps_tester()
        opsummering.rotary_encoder = rotary_encoder_tester()
        opsummering.lcd = lcd_tester()
        opsummering.EEPROM = i2c_ping_EEPROM()
        afslut()
