https://www.sunfounder.com/2-channel-dc-5v-relay-module-with-optocoupler-low-level-trigger-expansion-board.html
https://www.sunfounder.com/products/2channel-relay-module?_pos=2&_sid=e9e547828&_ss=r

Neu (Januar 2026):
https://www.sunfounder.com/products/2-channel-dc-5v-relay-module-low-level-trigger?_pos=1&_sid=e24d405c0&_ss=r
https://www.berrybase.de/en/5v-2-channel-relay-module
https://www.az-delivery.de/en/products/2-relais-modul?srsltid=AfmBOopC-4R5pFhHG3L81nVzHLQj-LeupisplH0Djn8kWX-mipE_gvgA
https://www.reichelt.de/de/de/shop/produkt/entwicklerboards_-_relais-modul_2_channel_5_v-282568

GPIO Raspberry Pi
https://tutorials-raspberrypi.de/raspberry-pi-gpio-erklaerung-beginner-programmierung-lernen/
https://indibit.de/raspberry-pi-die-gpio-schnittstelle-grundlagenbelegung/
https://pinout.xyz/#


import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)   # GPIO_17 == PIN 11 == IO3
			   # GPIO_27 == PIN 13 == IO2
			   # GPIO_4  == PIN 7  == IO1
GPIO.output(17, GPIO.HIGH)
GPIO.output(17, GPIO.LOW)


Relais anschließen auf Heizungsplatine:

- GND
- +5V
- IO1

TODO: falls Probleme mit der Seriellen Schnittstelle zur Heizungsplatine auftreten -> Heizungsplatine kurz ausschalten und heizung.py neu starten...

ACHTUNG: ggf. gibt es einen Fehler im reconnect() -> dort wird nach ttyUSB gesucht und nicht nach /dev/serial0 !!! -> find_all_usb_rs232_devices()