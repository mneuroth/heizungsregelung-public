https://www.sunfounder.com/2-channel-dc-5v-relay-module-with-optocoupler-low-level-trigger-expansion-board.html
https://www.sunfounder.com/products/2channel-relay-module?_pos=2&_sid=e9e547828&_ss=r

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

