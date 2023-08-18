from machine import SoftI2C, Pin
import eeprom_24xx64
from gpio_lcd import GpioLcd

i2c = SoftI2C(scl=Pin(18), sda=Pin(19))
e = eeprom_24xx64.EEPROM_24xx64(i2c)


def lcd_skriv_navn():
    # Create the LCD object
    lcd = GpioLcd(rs_pin=Pin(27), enable_pin=Pin(25),
                  d4_pin=Pin(33), d5_pin=Pin(32), d6_pin=Pin(21), d7_pin=Pin(22),
                  num_lines=4, num_columns=20)
    lcd.clear()
    lcd.putstr("Velkommmen til KEA's")
    lcd.move_to(4, 1)
    lcd.putstr("IT-Teknolog")    
    lcd.move_to(4, 2)
    lcd.putstr("Uddannelse! :)")
    lcd.move_to(0, 3)
    lcd.putstr(e.read_string(8000))
    
navn = e.read_string(8000)

if navn[0] != '\xff':
    lcd_skriv_navn()
    
    
