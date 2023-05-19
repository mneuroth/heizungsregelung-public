"""
	Simple script to read infos via RS232 from heating board (Heizungsplatine).
"""

import heizung
import serial
import time

def execute_cmd(cmd):
    aPlatine = heizung.HeatingControlBoard("COM7",baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=5.0)
    print(aPlatine)
    aPlatine.write(cmd)
    result = aPlatine.readline()
    print(f"RESULT of command {cmd} -> {result}")
    aPlatine.close()
    aPlatine.open()
    aPlatine.write("READ_TEMP;")
    result = aPlatine.readline()
    print(f"RESULT of command {cmd} -> {result}")
    aPlatine.close()

execute_cmd("VERSION;")
while True:
    execute_cmd("READ_TEMP;")
    time.sleep(0.5)
