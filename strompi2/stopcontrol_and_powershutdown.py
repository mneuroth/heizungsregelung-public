#!/usr/bin/python
#!/usr/bin/python
# Initialisierung
import RPi.GPIO as GPIO
import time
import os
try:
    import urllib2
except ModuleNotFoundError as exc:
    import urllib.request as urllib2

def stop_heizung():
    response = urllib2.urlopen("http://127.0.0.1/stop")
    html = response.read()
    return html

GPIO.setmode(GPIO.BCM)
 
 
# Hier den entsprechden GPIO-PIN auswaehlen
GPIO_TPIN = 21
 
print( "Sicheres Herunterfahren bei Stromausfall (CTRL-C zum Schliessen)" )
 
# Set pin as input
GPIO.setup(GPIO_TPIN,GPIO.IN,pull_up_down = GPIO.PUD_DOWN)      # Echo
 
Current_State  = 0
Previous_State = 0
 
try:
 
  print( "Warte auf Initialisierung der Spannungsversorgung..." )
 
  
  while GPIO.input(GPIO_TPIN)==1:
    Current_State  = 0
 
  print( "  Bereit" )
 
  
  while True :
 
    Current_State = GPIO.input(GPIO_TPIN)
 
    if Current_State==1 and Previous_State==0:
      print( "ACHTUNG: STROMAUSFALL !!!" )
      Previous_State=1
      print( "stoppe Heizungsregelung..." )
      stop_heizung()
      time.sleep(10)
      print( "starte shutdown..." )
      # while loop in start_pi.sh will restart heating control again ! 
      os.system("sudo shutdown -h now")
    elif Current_State==0 and Previous_State==1:
      Previous_State=0
 
    time.sleep(0.01)
 
except KeyboardInterrupt:
  print( "  Quit" )
  GPIO.cleanup()
