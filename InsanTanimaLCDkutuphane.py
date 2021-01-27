#Raspberyy PI nin 2x16 LCD modulunu kullanabilmesi icin gerekli olan
#kutuphane tanimlamasi yapiliyor.

import RPi.GPIO as GPIO
import os
import time
from time import gmtime, strftime


#LCD'nin kullanacagi GPIO pinlerinin tanimi yapiliyor.
LCD_RS = 7
LCD_E  = 8
LCD_D4 = 25
LCD_D5 = 24
LCD_D6 = 23
LCD_D7 = 18


# LCD ye ait sabitler tanımlanıyor.
LCD_WIDTH = 16  # Satır basina dusen maksimum karakter sayisi
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # 1. Satir için LCD yea ait RAM Adreslemesi
LCD_LINE_2 = 0xC0 # 2. Satir için LCD yea ait RAM Adreslemesi

# Zaman Sabitleri
E_PULSE = 0.0005
E_DELAY = 0.0005


#GPIO pinleri ayarlaniyor.
def pinIoAyarla():
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BCM)       # Mod ayarlaniyor
  GPIO.setup(LCD_E, GPIO.OUT)  # E
  GPIO.setup(LCD_RS, GPIO.OUT) # RS
  GPIO.setup(LCD_D4, GPIO.OUT) # DB4
  GPIO.setup(LCD_D5, GPIO.OUT) # DB5
  GPIO.setup(LCD_D6, GPIO.OUT) # DB6
  GPIO.setup(LCD_D7, GPIO.OUT) # DB7


def main():
  
  #Pinler ayarlaniyor
  pinIoAyarla()

  # LCD ye ait ilk ayarlamalar (Initialise) yapiliyor
  lcd_init()

#LCD ye ait ilk tanimlamalari yapan metot
def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) 
  lcd_byte(0x32,LCD_CMD) 
  lcd_byte(0x06,LCD_CMD) 
  lcd_byte(0x0C,LCD_CMD) 
  lcd_byte(0x28,LCD_CMD) 
  lcd_byte(0x01,LCD_CMD) 
  time.sleep(E_DELAY)

#LCD ye ait mod ve diger ayarlamalari yapan metot
def lcd_byte(bits, mode):

  GPIO.output(LCD_RS, mode) # RS

  #Yuksek seviyeli bitler
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)

  # Enable pin toggle ediliyor
  lcd_toggle_enable()

  #Dusuk seviyeli bitler
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)

  # Enable pin toggle ediliyor
  lcd_toggle_enable()

def lcd_toggle_enable():
  # Enable islemi gerceklestiriliyor
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)

#LCD ye mesaj yazdirmaya yarayan metot
def lcd_string(message,line):
  # Mesaj string e cast(donusturuluyor) ediliyor
  message = str(message)
  # Mesaj LCD ye gonderiliyor
  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

if __name__ == '__main__':

  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)
    lcd_string("Gule gule",LCD_LINE_1)
    GPIO.cleanup()