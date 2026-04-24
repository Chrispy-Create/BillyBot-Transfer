#!/usr/bin/env python3
import socket
import time
import board
import digitalio
import adafruit_character_lcd.character_lcd as characterlcd

# Initialize your LCD here (pins will vary by wiring):
lcd_rs = digitalio.DigitalInOut(board.D7)
lcd_en = digitalio.DigitalInOut(board.D8)
lcd_d4 = digitalio.DigitalInOut(board.D25)
lcd_d5 = digitalio.DigitalInOut(board.D24)
lcd_d6 = digitalio.DigitalInOut(board.D23)
lcd_d7 = digitalio.DigitalInOut(board.D18)
lcd = characterlcd.Character_LCD_Mono(
    lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7,
    columns=16, lines=2
)

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except:
        ip = "No Net"
    finally:
        s.close()
    return ip

if __name__ == "__main__":
    lcd.clear()
    ip = get_ip()
    lcd.message = f"Pi IP:\n{ip}"
    # Keep it displayed
    while True:
        time.sleep(60)
