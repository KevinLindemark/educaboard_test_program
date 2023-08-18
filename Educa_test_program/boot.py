from machine import I2C, Pin, PWM
import eeprom_24xx64
from gpio_lcd import GpioLcd


#########################################################################
# LCD KONTRAST CONFIGURATION

pin_lcd_contrast = 23
contrast_level = 250                        # Varies from LCD to LCD and wanted contrast level: 0-1023
lcd_contrast = PWM(Pin(pin_lcd_contrast))   # Create PWM object from a pin
lcd_contrast.freq(440)                      # Set PWM frequency
lcd_contrast.duty(contrast_level)

#########################################################################
# LCD KONTRAST CONFIGURATION SLUT

i2c = I2C(0)
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
    
navn = e.read_byte(8000)

if navn < 20:
    lcd_skriv_navn()      