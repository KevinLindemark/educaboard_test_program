from machine import Pin, ADC, UART
from time import sleep
from gps_GPGGA_GPZDA import GPS_GPGGA_GPZDA
from rotary_encoder import rotary_encoder_tester
from gpio_lcd import GpioLcd

class opsummering:
    led = ""
    potentiometer = ""
    knapper = ""
    LMT84 = ""
    gps = ""
    rotary_encoder = ""
    lcd = ""
    def status():
        print(f"\n{opsummering.led}\n{opsummering.potentiometer}\n{opsummering.knapper}\n{opsummering.LMT84}\n{opsummering.gps}\n{opsummering.rotary_encoder}\n{opsummering.lcd}")
   
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
                print("GPS virker!")
                opsummering.gps = "GPS virker"
                break
            else:
                print("GPS, virker ikke")
                opsummering.gps = "GPS virker ikke"
                break
        count +=1
 
def led_tester(g_val, y_val, r_val):
    print("Tester LED'er. lyser en grøn, gul og rød LED på educaboardet? (ja/nej)\n")
    yellow = 26                          # Direct connection
    green = 12                         # Put jumper between JP1-MISO and JP6-GP2
    red = 13                            # Put jumper between JP1-MOSI and JP6-GP3

    g = Pin(green, Pin.OUT)
    y = Pin(yellow, Pin.OUT)
    r = Pin(red, Pin.OUT)

    r.value(g_val)
    g.value(y_val) # yellow is active low
    y.value(r_val)
    svar_led = input()
    if svar_led.lower() == "nej":
     print("Gennemgå LED kredsløbet for fejl og prøv igen. Lukker testprogram")
     print("Husk at forbinde \"JP1-MISO <-> JP6-GP2\" and \"JP1-MOSI <-> JP6-GP3\"")
     opsummering.led = "Der er problemer med LED'er"
    else:
     opsummering.led = "LED'er virker"
    
     
def potentiometer_tester():
    print("Tester potentiometer. Vises værdier mellem 0 og 4095 når der skrues på potentiometeret? (ja/nej)\n")
    sleep(4)
    pot = ADC(Pin(34))
    count = 0
    while count < 5:
        print(pot.read())
        count +=1
        sleep(0.1)
    print("Tester potentiometer. Vises værdier mellem 0 og 4095 når der skrues på potentiometeret? (ja/nej/forfra)\n")
    svar_potentiometer = input().lower()
    if svar_potentiometer == "nej":
      print("Gennemgå LED kredsløbet for fejl og prøv igen. Lukker testprogram")
      opsummering.potentiometer = "Der er problemer med potentiometeret"
    elif svar_potentiometer == "forfra":
      print("Tester potentiometer igen, kig i shell og skru på potentiometeret")
      sleep(3)
      potentiometer_tester()
    else:
      opsummering.potentiometer = "Potentiometeret virker"

def knap_tester():
    led1 = Pin(26, Pin.OUT)
    led1.value(0)
    print("Tester tryknapperne, når pb1 trykkes bør LED1 lyse, og når PB2 holdes nede bør LED1 blinke")
    pb1 = Pin(4, Pin.IN)                 # External pull-up and debounce
    pb2 = Pin(0, Pin.IN)                 # Direct connection with pull-up thus inverted
    #pb2 = Pin(4, Pin.IN, Pin.PULL_DOWN)
    count = 0
    while count < 10:
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
      opsummering.knapper = "Trykknapper virker"
    elif svar_knap == "forfra":
      knap_tester()
    else:
      opsummering.knapper = "Trykknapper virker ikke"

def afslut():
    afslut = input("test er slut - afslut og se opsummering? ja/nej\n")
    if afslut == "ja":
      opsummering.status()
      global testing
      testing = False
    else:
      print("Kører testen igen")

def lmt84_tester():
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
    print("ADC: %3d -> %2.1f °C" % (adcVal, temp))
    svar_temp = input("Vises nogenlunde korrekt temperatur fra LMT84 sensor? ja/nej\n").lower()
    if svar_temp == "nej":
        opsummering.LMT84 = "LMT84 virker ikke"
    else:
        opsummering.LMT84 = "LMT84 virker"

def lcd_tester():
    print("LCD 20x4 test. kan du se teksten på Educa boardets display? ja/nej?")

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
        return "lcd display virker"
    else:
        print("Prøv at juster kontrasten")
        return "lcd display virker ikke"

testing = True
if __name__ == "__main__":
    while testing:
      led_tester(1, 0, 1)
      potentiometer_tester() 
      knap_tester()
      lmt84_tester()
      gps_tester()
      opsummering.rotary_encoder = rotary_encoder_tester()
      opsummering.lcd = lcd_tester()
      afslut()
  
  
 


