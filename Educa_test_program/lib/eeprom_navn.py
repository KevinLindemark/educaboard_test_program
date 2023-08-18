from machine import SoftI2C, Pin
import eeprom_24xx64

def eeprom_student_navn(): 
    i2c = SoftI2C(scl=Pin(18), sda=Pin(19))
    e = eeprom_24xx64.EEPROM_24xx64(i2c)

    navn = e.read_string(8000)
    if navn[0] == '\xff':
        studerendes_navn = input("Skriv dit navn og efternavn og tryk enter - Max 32 tegn\n")
        e.write_string(8000, studerendes_navn)
        print("Dit navn: ",e.read_string(8000))
