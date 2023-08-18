from machine import I2C, Pin
import eeprom_24xx64
def eeprom_student_navn(): 
    i2c = I2C(0)
    e = eeprom_24xx64.EEPROM_24xx64(i2c)

    navn_byte = e.read_byte(8000)
    if navn_byte > 20:
        studerendes_navn = input("Skriv dit navn og efternavn og tryk enter - Max 20 tegn\n")
        while len(studerendes_navn) > 20:
            print("Uyldigt navn, prøv igen og skriv det kortere! (Max 20 tegn)")
            studerendes_navn = input("Skriv dit navn og efternavn og tryk enter - Max 20 tegn\n")
            
        else:    
            e.write_string(8000, studerendes_navn)
            print("Dit navn: ",e.read_string(8000))