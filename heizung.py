# -*- coding: utf-8 -*-
r""" 
   ****************************
   * Heating control program. *
   ****************************

   Test mode:
   ----------
    >python heizung.py -t

   All options:
   ------------
    > python heizungsregelung.py -h
    >
    > heating control programm, available options:
    > -s : silent modus, no debugging output
    > -i : use simulation port 8008 for http
    > -p : use default port 80 for http
    > -t : test without hardware (simulate hardware)
    > -e : use exclusive access modus for RS232 interface
    > -c : run with history cache
    > -d : run without history cache
    > -w : disable watchdog in python program
    > -h : show this help   

   Info:
   - Values of manual switches are cached in directory /sdcard/cache or ./cache
   - Plot history is cached in directory /sdcard/cache or ./cache
   - State of manual switches are stored in directory /sdcard/persistence or ./persistence

   ************************************************************************
   
    Class Overview:
    ===============
    
    object
      |
    Device
      |
    Rs232Device
      |
    ArduinoMega   ConradMultiRelais   HeatingControlBoard   # used by SignalProcessor instances
      
    
    object
      |
    ResouceScope
    

    list
      |
    ControlEngine
      
    
    object
      |
    SignalProcessor
      |       |
      |     TemperatureMeasurement  RelaisMeasurement  DataSource  ThermicalSolarCollectorControl            HeatingControl         
      |                                                  |                                                     |
      |                                                TickCounterSource  TemperatureSource  LightSource     DesiredValueForHeatingMixerControl
      |
    HeatingMixerControl  HeatPumpControl  VentilationControl  HeatPumpSolarValveControl  ManualSwitch  ManualSwitchAutoReset  SwitchRelais  ThreeStateSwitchRelais  ValueSwitch  NotNode  OrNode  AndNode  OperatingHoursCounter  EnableTimer  Watchdog  Timeline
    
    =======================================================================
    
    ControlEngine = [SignalProcessor,...]	# manages a list of SignalProcessor objects, is the time processor --> clock_tick()
    
    SignalProcessor 			        # has InputSignals==SignalProcessor and a Timeline (history output values) (OutputSignals only for debugging == Graph Analysis)
                                        # uses the InputSignals to get values which will be used in the clock_tick() method to calculate my own (output) clock_tick-value
                                        # this (output) clock_tick-value could be used as input value for other SignalProcessor objects
                                     
    Example:
    --------    
                                                                      RelaisMeasurement
                                                                              ^
                                                                              .
    TemperatureSource (SOLAR_SLVF) --\                                        . uses
                                      \ 					                  .
    TemperatureSource (SOLAR_KVLF) --> ThermicalSolarCollectorControl --> SwitchRelais (SWITCH_MOTOR_SOLAR)
       .                                                                 /
       . uses                                            ManualSwitch --/ (MANUAL_SWITCH_MOTOR_SOLAR)
       .
       V
    TemperatureMeasurement
    
    
    TemperatureMeasurement   		    # has no logical input, just for temperature measurement in a clock_tick() (side effect!) 
    
    ThermicalSolarCollectorControl		# uses the two input nodes to get values for temperatures and calculates an output value 
                                          (nothing will be done with this value, except any other node has this node as an input node,
                                           in this case the values of this node will be used in the other node to calculate a value
                                           for the other node)
                                           
    -----------------------------------------------------------------------

    QtControlGui   <---http--->   SimpleServer    ---queue--->   ControlProcess
                                    Py-Proc      <---queue---       Py-Proc
                                                    
                                                 MainProcess
                                                   Py-Proc
                                                   
 
    Design-Info:
    ============

    the heizung application has 3 different (asynchroniously running) loops:
                                                                                                  |
       Control-Loop (Main-Thread)   <----------->  HttpCommunicationThread (Worker-Thread)    <=======>    HttpServerThread (Worker-Thread)
                                     Vars & Lock                                                Queue
                                                                                                  |
                                                                                       option (A): two different processes
                                                                                       option (B): one process, but multipe threads

"""

import sys
import os
import os.path
import io
#import getpass
import glob
import datetime
import time
import functools
import math
import traceback
import signal

import serial
import requests

from functools import wraps

import pipeprocessing

import socket
import selectors
import libserver

from file_utils import *

try:
    from StringIO import StringIO
except:
    from io import StringIO

g_bUseThreads = True    # True  --> run control and http-server in one process in different threads (communicate via Queue)
                        # False --> run control and http-server in different processes. the control procress has two threads for control-loop and http-communication-loop (to communicate with http-server) (communicate via multiprocessing.Queue)

if not g_bUseThreads:
    try:
        from multiprocessing import Queue, Process #, freeze_support
    except ImportError as e:
        print( e )
        print( "using threading module !" )
        from threading import Thread as Process    
        from queue import Queue
        Process.is_alive = Process.isAlive   
        #freeze_support = lambda : True
else:
    try:
        from queue import Queue
    except:
        # for Raspberry Pi Python 2.7.13
        from Queue import Queue

import threading

if  sys.version_info.major==3:
    import _thread as thread    # for python 3
else:
    import thread               # for python 2

# *************************************************************************

LAYER_UNKNOWN = 'LAYER_UNKNOWN'
LAYER_INPUT = 'LAYER_INPUT'
LAYER_MANUAL_SWITCH = 'LAYER_MANUAL_SWITCH'
LAYER_CONTROL = 'LAYER_CONTROL'
LAYER_OUTPUT = 'LAYER_OUTPUT'

g_bCanControlHeatingControlBoard = False
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    g_bCanControlHeatingControlBoard = True
except ImportError as e:
    print( "Warning: Not running on Raspberry Pi, can not control HeatingControlBoard !" )
    
def SwitchHeatingControlBoard(bOn):
    if g_bCanControlHeatingControlBoard:
        GPIO.output(4, GPIO.LOW if bOn else GPIO.HIGH)
        return True
    else:
        # function not supported
        return False

# *************************************************************************

def is_raspberrypi():
    try:
        with io.open('/sys/firmware/devicetree/base/model', 'r') as m:
            if 'raspberry pi' in m.read().lower():
                return True
    except Exception: 
        pass
    return False

# only fallback constants, devices will be queried
if os.name=="posix" and sys.platform=="darwin":
    # for MacOS:
    DEVICE_RS232_ARDUINO = "/dev/tty.usbserial-A900acbK"
    DEVICE_RS232_RELAIS  = "/dev/tty.usbserial-A800em6n"
    DEVICE_RS232_HEATINGCONTROLBOARD = "/dev/tty.usbserial-A900acbK"
elif os.name=="posix":
    # for Friendlyarm:
    DEVICE_RS232_ARDUINO = "/dev/ttySAC2"
    DEVICE_RS232_RELAIS  = "/dev/ttySAC1"
    DEVICE_RS232_HEATINGCONTROLBOARD = "/dev/ttySAC2"
    if os.uname()[1]=="raspberrypi" or os.uname()[1]=="heizungsregelung" or is_raspberrypi():
        # for Raspberry Pi:
        DEVICE_RS232_HEATINGCONTROLBOARD = "/dev/serial0"
        DEVICE_RS232_ARDUINO = "/dev/serial0"
        DEVICE_RS232_RELAIS  = "/dev/serial0"
elif os.name=="nt":
    # for Windows:
    DEVICE_RS232_ARDUINO = "COM12"   # org: 2
    DEVICE_RS232_RELAIS  = "COM13"   # org: 3
    DEVICE_RS232_HEATINGCONTROLBOARD = "COM7"

# *************************************************************************
__version__ = "2.8.4"
__date__    = "9.11.2024"
# *************************************************************************
START_YEAR = 2010

g_support_heatpump_via_pv = True

g_bDebug = True
g_bSimPort = False
g_bUseHttpDefaultPort = False
g_bTest = False
g_bExclusiveRs232 = False
g_bUseCache = True
g_bUseWatchdog = False
g_bDumpDependencyGraph = False
g_actUsbRs232Devices = []

def is_debug():
    return g_bDebug

def set_debug(bValue):
    global g_bDebug
    g_bDebug = bValue

def set_test(bValue):
    global g_bTest
    g_bTest = bValue

def set_exclusive_rs232(bValue):
    global g_bExclusiveRs232
    g_bExclusiveRs232 = bValue

def set_sim_port(bValue):
    global g_bSimPort
    g_bSimPort = bValue

def set_default_http_port(bValue):
    global g_bUseHttpDefaultPort
    g_bUseHttpDefaultPort = bValue

def set_use_cache(bValue):
    global g_bUseCache
    g_bUseCache = bValue

def set_use_watchdog(bValue):
    global g_bUseWatchdog
    g_bUseWatchdog = bValue
    
def set_dump_dependency_graph(bValue):
    global g_bDumpDependencyGraph
    g_bDumpDependencyGraph = bValue

def find_win32_rs232_devices():
    ret = []
    for i in range(255):
        try:
            aCOM = serial.Serial("COM"+str(i+1))    # PATCH for new pySerial version
            ret.append("COM"+str(i+1))
        except serial.SerialException:
            pass
    return ret
    
def get_mapped_usb_rs232_devices():
    return g_actUsbRs232Devices

def update_mapped_usb_rs232_device(sOldName,sNewName):
    aLst = get_mapped_usb_rs232_devices()
    if sOldName in aLst:
        aLst.remove(sOldName)
    aLst.append(sNewName)

def find_all_usb_rs232_devices():
    if os.name == 'nt':
        return find_win32_rs232_devices()
    elif os.name == 'posix' and sys.platform=='darwin':
        return glob.glob("/dev/tty.usbserial*")
    elif os.name == 'posix':
        return glob.glob("/dev/ttyUSB*")
    else:
        return []        

def get_new_not_used_usb_rs232_devices(sMyOldRs232Name):
    lstMaybeUsedRs232 = []
    # hole alle /dev/ttyUSB* namen aus dem aktuellen daten
    # ausser dem uebergebenen namen, der gerade ungueltig geworden ist
    for key in get_mapped_usb_rs232_devices():
        if key!=sMyOldRs232Name:
            lstMaybeUsedRs232.append(key)
    # entferne aus der liste der aktuell verfuegbaren USB devices die wahrscheinlich verwendeten devices
    ret = []
    for e in find_all_usb_rs232_devices():
        if not (e in lstMaybeUsedRs232):
            ret.append(e)
    return ret
    
def find_rs232_device_for_hardware_class(aLstDevices,aHardwareClass,baudrate,bytesize,parity,stopbits,timeout):
    for e in aLstDevices:
        aHardware = aHardwareClass(e,baudrate,bytesize,parity,stopbits,timeout)
        if aHardware.is_ok() and aHardware.ping():
            update_mapped_usb_rs232_device("",e)
            aLstDevices.remove(e)
            return (aLstDevices,aHardware)
        else:
            aHardware.close()
            aHardware = None
    return (aLstDevices,None)

# see: https://stackoverflow.com/questions/15389768/standard-deviation-of-a-list
def stddev(lst):
    try:
        if len(lst)>0:
            mean = float(sum(lst)) / len(lst)
            return math.sqrt(float(reduce(lambda x, y: x + y, map(lambda x: (x - mean) ** 2, lst))) / len(lst))
    except:
        return -1.0
    return 0.0

def _node_type_to_shape(node_type):
    if node_type == LAYER_INPUT:
        return 'ellipse'
    elif node_type == LAYER_CONTROL:
        return 'parallelogram'
    elif node_type == LAYER_MANUAL_SWITCH:
        return 'house'
    elif node_type == LAYER_OUTPUT:
        return 'hexagon'
    return 'box'
   
def _node_type_to_color(node_type):
    if node_type == LAYER_INPUT:
        return 'orange'
    elif node_type == LAYER_CONTROL:
        return 'blue'
    elif node_type == LAYER_MANUAL_SWITCH:
        return 'red'
    elif node_type == LAYER_OUTPUT:
        return 'green'
    return 'black'
   
# *************************************************************************
class Device(object):
    
    def is_ok(self):
        """ Is the device ready to use ? """
        return False

    def ping(self):
        """ Reacts the device for a message ? """
        return False
        
    def finish(self):
        """ Helper for resetting the hardware: 
            first finish (free) resources and than reconnect to resource.
        """
        pass

    def run_persistence(self):
        """ CTRL-C handling: 
            write persistent data
        """
        pass

# *************************************************************************
class ResouceScope(object):

    """
        Implements a Context Manager for a resource.
        Used for accessing a Device using a RS232 resource.
        RS232 port will be opened if needed and closed if not needed anymore.
    """

    def __init__(self, aResource, use_exclusive=False):
        #print("INIT ResourceScope")
        self.aResource = aResource
        self.use_exclusive = use_exclusive

    def __enter__(self):
        #print("__enter__ ResourceScope")
        if not self.aResource.is_open():
            self.aResource.open()
# TODO: ggf. reset() nach open() aufrufen?            
        return self.aResource
    
    def __exit__(self, type, value, traceback):
        #print("__exit__ ResourceScope")
        self.aResource.reset()
        if self.aResource.is_open() and not self.use_exclusive:
            self.aResource.close()

# *************************************************************************
def generic_retry_on_fail(func, fcn_log, retries_on_fail=10, retry_after_seconds=5, block='Generic Retry'):
    """
    Generic retry decorator
    """

    @wraps(func)
    def wrapper(*args, **kwargs):
        succeeded = False
        retry_count = 0
        for numbers_to_retry in range(retries_on_fail, 0, -1):
            try:
                retry_count += 1
                result = func(*args, **kwargs)
                return result
            except Exception as e:
                fcn_log(f"[{block}] failed operation, as '{e}', still [{numbers_to_retry}] time(s) to retry")
                time.sleep(retry_after_seconds)
                continue
        if not succeeded:
            raise Exception(f"[{block}] failed operation after {retries_on_fail} times retried!")

    return wrapper

open_retry_repeater = lambda fcn: generic_retry_on_fail(fcn, print, 12, 5, block='RS232 open')

# *************************************************************************
class Rs232Device(Device):
    
    """
        Wrapper for pyserial RS232 object, 
        could also be used as simulation object if opening of RS232 port fails.
    """

    def __init__(self,sRs232Name,baudrate=1200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=None):  #19200
        super(Rs232Device,self).__init__()
        self.flushInput = lambda : self.try_func(self._flushInput)   
        self.flushOutput = lambda : self.try_func(self._flushOutput)   
        self.reset = lambda : self.try_func(self._reset)   
        self.read = lambda v : self.try_func(self._read,v)   
        self.readline = lambda : self.try_func(self._readline)   
        self.write = lambda v : self.try_func(self._write,v)   
        self.sRs232Name = sRs232Name
        self.aRs232 = serial.Serial(sRs232Name,baudrate,bytesize,parity,stopbits,timeout)
        append_to_logfile("CONSTRUCTOR Rs232Device() "+str(sRs232Name)+" "+str(self),is_debug())
#            self.reset()
            
    # WARNING: DO NOT USE __del__, because it disturbs the garbage collector while freeing resources ! see python documentation
    #def __del__(self):
    #    print "__DEL__","open=","None" if self.aRs232==None else self.aRs232.isOpen(),self,self.aRs232
    #    append_to_logfile("!!! DESTRUCTOR Rs232Device() "+str(self.sRs232Name)+" "+str(self),is_debug())
    #    if self.aRs232:
    #        self.aRs232.close()
    #    self.aRs232 = None

    def try_func(self,fcn,*args):
        try:
            return fcn(*args)
        except Exception as e:     # serial.SerialException
            append_to_logfile(str(datetime.datetime.now())+": Exception (0) in try_func(): "+str(e)+" for function: "+str(fcn)+" args: "+str(args),is_debug())
            try:
                self.reconnect()  
            except Exception as e2:
                append_to_logfile(str(datetime.datetime.now())+": Exception (1) in try_func(): "+str(e2),is_debug())
                try:
                    self.search_renamed_port() 
                except Exception as e3:
                    append_to_logfile(str(datetime.datetime.now())+": Exception (2) in try_func(): "+str(e3),is_debug())
                    # if this following call still raises an exception 
                    # we must reconntect both rs232 devices again, because
                    # most probably we have two new usb-rs232 device names !
                    self.check_all_ports()
            return fcn(*args)

    def reopen(self):
        try:
            #print("reopen")
            self.reconnect()  
        except Exception as e2:
            append_to_logfile(str(datetime.datetime.now())+": Exception (1) in reopen(): "+str(e2),is_debug())
            try:
                self.search_renamed_port() 
            except Exception as e3:
                append_to_logfile(str(datetime.datetime.now())+": Exception (2) in reopen(): "+str(e3),is_debug())
                # if this following call still raises an exception 
                # we must reconntect both rs232 devices again, because
                # most probably we have two new usb-rs232 device names !
                self.check_all_ports()

    def _reopen(self,lstUsbRs232Devices):
        #print("_reopen")
        sOldRs232Name = self.sRs232Name
        for sNewPortName in lstUsbRs232Devices:
            self.close()
            time.sleep(3.0)
            self.setPort(sNewPortName)
            self.open()
            if self.is_open():
                # check if this device is the expected device (== same type as before reconnect())
                if self.ping():
                    # update global data !
                    update_mapped_usb_rs232_device(sOldRs232Name,sNewPortName)
                    return True
        return False    

    def check_all_ports(self):
        """
            Check all existing serial ports for my original device,
            used after a reset of the USB Hub
        """
        append_to_logfile("CHECK_ALL_PORTS "+str(self.sRs232Name)+" "+str(self),is_debug())
        if not self._reopen(find_all_usb_rs232_devices()):
            raise Exception("Device is not the expected device (2) !")

    def search_renamed_port(self):
        """
            Try to find another serial port which might be the old one.
            Could be called if a USB error occured and the hub is reset.
        """
        append_to_logfile("SEARCH_RENAMED_PORT "+str(self.sRs232Name)+" "+str(self),is_debug())
        if not self._reopen(get_new_not_used_usb_rs232_devices(self.sRs232Name)):
            raise Exception("Device is not the expected device (1) !")
        
    def reconnect(self):
        """
            Reconnect with serial port
        """
        self.reset()
        append_to_logfile(str(datetime.datetime.now())+": RECONNECT "+str(self.sRs232Name)+" "+str(self),is_debug())
        if not self._reopen([self.sRs232Name]):
            raise Exception("Device is not the expected device (0) !")
        self.reset()
        
    def finish(self):
        #print "FINISH RS232 Device",self,self.aRs232
        #print("FINSIH")
        self.close()
        self.aRs232 = None
        
    def close(self):
        if self.aRs232:
            #print("close")
            self.aRs232.close()

    @open_retry_repeater
    def open(self):
        if self.aRs232:
            #print("open")
            self.aRs232.open()

    def is_open(self):
        if self.aRs232:
            #print("is_open")
            return self.aRs232.is_open
        return False

    def setPort(self,sPort):
        if self.aRs232:
            #print("setport",sPort)
            self.aRs232.setPort(sPort)

    def _flushInput(self):
        if self.aRs232:
            #print("flushInput")
            self.aRs232.reset_input_buffer()
               
    def _flushOutput(self):
        if self.aRs232:
            #print("flushOutput")
            self.aRs232.reset_output_buffer()

    def _reset(self):
        if self.aRs232:
            #print("reset")
            self.aRs232.reset_input_buffer()
            self.aRs232.reset_output_buffer()
            
    def _read(self,iSize):
        if self.aRs232:
            #print("read",iSize)
            ret = self.aRs232.read(iSize)
            return ret
        else:
            return ""

    def _readline(self):
        if self.aRs232:            
            try:
                #print("readline")
                ret = ""    
                ret = self.aRs232.readline()
                if isinstance(ret, bytes):
                    return ret.decode("utf-8") #, errors='ignore')
                return ret
            except UnicodeDecodeError as unierr:
                ret = ret.decode("utf-8", errors='ignore')
                # log is new since 4.2023 !
                append_to_logfile(str(datetime.datetime.now())+": INFO in _readline(0): "+str(unierr)+" read="+str(ret),is_debug())
# TODO -> Synchronisiere Zustand zwischen Heizungsregelung und Heizungsplatine !!! -> siehe Problem vom 19.1.2024 -> HeatingControlBoard.update_status()
                return ret
            except Exception as ex:
                # log is new since 4.2023 !
                append_to_logfile(str(datetime.datetime.now())+": EXCEPTION in _readline(1): "+str(ex)+" read="+str(ret.decode("utf-8", errors='ignore')),is_debug())
                return ""
        else:
            return ""

    def _write(self,sData):
        """
            returns number of writen bytes
        """
        if self.aRs232:
            #print("write")
            return self.aRs232.write(bytes(sData, "utf-8"))
        else:
            return 0

    def is_ok(self):
        return self.aRs232!=None           

# *************************************************************************

def do_three_switch(iChannelOpen,iChannelClose,iValue,fcn_switch_port):
    iValue = int(iValue)
    if iValue==-1:  # == Stop
        fcn_switch_port(iChannelOpen,False)
        fcn_switch_port(iChannelClose,False)
    elif iValue==0: # == Close
        fcn_switch_port(iChannelOpen,False)
        fcn_switch_port(iChannelClose,True)
    elif iValue==1: # == Open
        fcn_switch_port(iChannelClose,False)            
        fcn_switch_port(iChannelOpen,True)
    # new since 3.10.2010: handle Mixer Ticks/Pulses for tests
    elif iValue==2: # == Open-Tick / Pulse
        fcn_switch_port(iChannelClose,False)            
        fcn_switch_port(iChannelOpen,True)
        time.sleep(1.0)                # wait a moment
        fcn_switch_port(iChannelOpen,False)
    elif iValue==3: # == Close-Tick / Pulse
        fcn_switch_port(iChannelOpen,False)
        fcn_switch_port(iChannelClose,True)
        time.sleep(1.0)                # wait a moment
        fcn_switch_port(iChannelClose,False)

# *************************************************************************
class HeatingControlBoard(Rs232Device):
    
    def __init__(self,sRs232Name,baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=None):
        super(HeatingControlBoard,self).__init__(sRs232Name,baudrate,bytesize,parity,stopbits,timeout)
        append_to_logfile("CONSTRUCTOR HeatingControlBoard() "+str(sRs232Name)+" "+str(self),is_debug())
        self.cache = [None for i in range(16)]

    def ping(self):
        with ResouceScope(self,g_bExclusiveRs232) as device:
            iCount = device.write("PING;")
            sData = device.readline()
            return sData.startswith("PONG;")        
        
    def read_temperatures(self):
        with ResouceScope(self,g_bExclusiveRs232) as device:
            device.write("READ_TEMP;")
            return device.readline()

    def update_status(self):
        with ResouceScope(self,g_bExclusiveRs232) as device:
            device.write("READ_RELAIS;")
            result = device.readline()
# TODO -> if read-status != current-status -> write current-status to board
            #print "update_status", result
            # process something like: Rel1=0;Rel2=1;Rel3=1;...;Rel12=0;
# TODO --> update the cache...        
            return 0

    def three_switch_port_fcn(self,iChannelOpen,iChannelClose):
        return functools.partial(self._three_switch,iChannelOpen,iChannelClose)
        
    def switch_port_fcn(self,iChannel):
        return functools.partial(self._switch_port,iChannel)
        
    def switch_port_for_not_value_fcn(self,iChannel):
        return functools.partial(self._switch_port_for_not_value,iChannel)
        
    def switch_virtual_port_fcn(self,iVirtualChannel):
        return functools.partial(self._switch_virtual_port,iVirtualChannel)
        
    def _three_switch(self,iChannelOpen,iChannelClose,iValue):
        do_three_switch(iChannelOpen,iChannelClose,iValue,self._switch_port)
        
    def _switch_port(self,iChannel,bValue):
        #print "_switch_port",iChannel,bValue
        sValue = "1" if bValue else "0"
        # write something like: SET_RELAIS:rel1=1;
        # REMARK: Hardware/firmware index=1..12 <==> python control program logical index=0..11  
        bWriteToHardware = self.cache[iChannel] != bValue
        if bWriteToHardware:
            self.cache[iChannel] = bValue
            with ResouceScope(self,g_bExclusiveRs232) as device:
                device.write("SET_RELAIS:rel"+str(iChannel+1)+"="+sValue+";")
                return device.readline()     
        else:
            # simulate original response
            return "OK:Rel"+str(iChannel)+"="+str(int(bValue))+"\n"
            
    def _switch_port_for_not_value(self,iChannel,bValue):
        self._switch_port(iChannel,not bValue)
        
    def _switch_virtual_port(self,iVirtualChannel,bValue):
        # simulate original response
        return "OK:Rel"+str(iVirtualChannel)+"="+str(int(bValue))+"\n"

# *************************************************************************
class ArduinoMega(Rs232Device):
    
    def __init__(self,sRs232Name,baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=None):
        super(ArduinoMega,self).__init__(sRs232Name,baudrate,bytesize,parity,stopbits,timeout)
        append_to_logfile("CONSTRUCTOR ArduinoMega() "+str(sRs232Name)+" "+str(self),is_debug())
        if self.is_ok():
            # after connecting with arduino via RS232 a reset of the arduino occures
            # give it a litle bit time for start of communication !
            time.sleep(3)
            #self.ping()
        #print "INIT Arduino",self,sRs232Name

    def read_temperatures(self):
        with ResouceScope(self,g_bExclusiveRs232) as device:
            device.write("READ_TEMP;")
            return device.readline()

    def ping(self):
        with ResouceScope(self,g_bExclusiveRs232) as device:
            iCount = device.write("PING;")
            sData = device.readline()
            return sData.startswith("PONG;")

# *************************************************************************
class ConradMultiRelais(Rs232Device):
    
    MIXER_TICK_TIME = 1.0   # until 7.11.2012: 3.0       # pulse time in seconds
    
    def __init__(self,sRs232Name,baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=None):
        super(ConradMultiRelais,self).__init__(sRs232Name,baudrate,bytesize,parity,stopbits,timeout)
        append_to_logfile("CONSTRUCTOR ConradMultiRelais() "+str(sRs232Name)+" "+str(self),is_debug())
        self.aActValues = None
        self.iErrorCount = 0
        self._initialize()
        self.update_status()
        #print "INIT ConradRelais",self,sRs232Name
        
    def ping(self):
        ret = self._write_message(0,0,0)
        if ret!=None and len(ret)>3:
            return ord(ret[0])==255 and ord(ret[1])==0 and ord(ret[2])==0
        return False

    def update_status(self):
        aActValues,iLenMsg = self._get_ports()
        if aActValues!=None:
            self.aActValues = aActValues
            self.iErrorCount = 0
        else:
            append_to_logfile("ERROR in relais.update_status() len="+str(iLenMsg)+" "+str(datetime.datetime.now()),is_debug())
            self.iErrorCount += 1
            if self.iErrorCount>5:      # until 26.10.2010 n=10, now n=5
                raise Exception("Error in update_status --> can not read port status !")
        return self.aActValues

    def three_switch_port_fcn(self,iChannelOpen,iChannelClose):
        return functools.partial(self._three_switch,iChannelOpen,iChannelClose)
        
    def switch_port_fcn(self,iChannel):
        return functools.partial(self._switch_port,iChannel)
        
    def switch_port_for_not_value_fcn(self,iChannel):
        return functools.partial(self._switch_port_for_not_value,iChannel)
        
    def switch_virtual_port_fcn(self,iVirtualChannel):
        return functools.partial(self._switch_virtual_port,iVirtualChannel)

    def _three_switch(self,iChannelOpen,iChannelClose,iValue):
        do_three_switch(iChannelOpen,iChannelClose,iValue,self._switch_port)
                
    def _switch_port(self,iChannel,bValue):
#(TODO --> ggf. nur senden falls sich zustand aendert ! ... ==> wohl nicht mehr notwendig seit dem die Probleme mit RS232/USB geloest sind --> durch UART verwenden
        ret = self._write_message(6 if bValue else 7,0,(1 << iChannel))
# TODO --> check result ? --> ret[2] == ports
        return True
        
    def _switch_port_for_not_value(self,iChannel,bValue):
        self._switch_port(iChannel,not bValue)
        
    def _switch_virtual_port(self,iVirtualChannel,bValue):
        # simulate original response
        return True

    def _get_ports(self):
        data = self._write_message(2,0,0)
        if data!=None and len(data)>2:
            return (int(ord(data[2])),len(data))
        return (None,-1 if data==None else len(data)) 
        
    def _initialize(self):
        ret = self._write_message(1,0,0)
# TODO --> check result ? --> ret[2] == firmware version == 11
        return True
        
    def _create_message(self,iByte0,iByte1,iByte2):
        iByte3 = iByte0 ^ iByte1 ^ iByte2
        return chr(iByte0)+chr(iByte1)+chr(iByte2)+chr(iByte3)
        
    def _write_message(self,iByte0,iByte1,iByte2):
        if self.aRs232:
            self.flushOutput()
            self.flushInput()
            #self.reset()
            count = self.write(self._create_message(iByte0,iByte1,iByte2))
            data = self.read(8)
            return data
        return None
        
# *************************************************************************
class ControlEngine(list):
    
    DELAY = 5.0         # old: 2.0 seconds between ticks 
    
    def run(self,sFileName,thread_communication_context,aLock):
        self._initialize()
        self.iCount = 0
        self.bStop = False
        self.aDataNodes = self._get_data_nodes()
        thread_communication_context['fcn_set_stop'] = self._set_stop
        thread_communication_context['fcn_poll_for_commands'] = self._poll_for_commands
        return self._run(sFileName,aLock)
    
    def _set_stop(self,value):
        self.bStop = value
        
    def finish(self):
        # delegate finish-call to all children
        for e in self:
            e.finish()

    def get_graph(self):
        sGraph = ""
        for signal_processor in self:
            if len(sGraph)>0:
                sGraph += "\n"
            sGraph += signal_processor.get_graph()
        return sGraph
    
    def get_dot_data(self):
        """
            Return data which can be used with graphviz (.dot)
        """
        sGraph = ""
        for signal_processor in self:
            #if len(sGraph)>0:
            #    sGraph += "\n"
            sGraph += signal_processor.get_dot_data()
        return sGraph

    def run_persistence(self):
        # delegate finish-call to all children
        for e in self:
            e.run_persistence()
        
    def _get_data_nodes(self):
        aData = {}
        for e in self:
            #if isinstance(e,TemperatureSource) or isinstance(e,SwitchRelais):
            aData[e.get_name()] = e
        return aData    

    def _get_act_data_from_data_nodes(self,iCount,bWithStdDev=False):
        aActData = {}
        aActData["ACT_TICK"] = iCount
        aActData["MASTER_ID"] = self.sMasterId
        for e in self.aDataNodes:
            if bWithStdDev:
                aActData[self.aDataNodes[e].get_name()] = (self.aDataNodes[e].get_value(), self.aDataNodes[e].get_stddev_value())
            else:
                aActData[self.aDataNodes[e].get_name()] = self.aDataNodes[e].get_value()
            # return additional data for the node, i. e. constants
            aAdditionalData = self.aDataNodes[e].get_additional_data_items()
            if aAdditionalData!=None:
                for key in aAdditionalData:
                    aActData[key] = aAdditionalData[key]
        return aActData
        
    def _get_history_for_data_node(self,args):
        if 'name' in args:
            sNodeName = args['name']
            if sNodeName in self.aDataNodes:
                return self.aDataNodes[sNodeName].get_history()
        return []
        
    def _update_data_for_data_nodes(self,aNewData):
        """
            Convert { "KEY" : value } into obj.get_name()=="KEY" --> obj.set_value(value)
        """
        iCount = 0
        for key in aNewData:
            # special handling for master_id --> is handled by the control engine object !
            if key=="MASTER_ID":
                self.sMasterId = aNewData[key]
            else:
                for e in self:
                    if key==e.get_name():                    
                        e.set_value(aNewData[key])
                        iCount += 1
        return (iCount,len(aNewData))

    def _poll_server_cmd_queue(self,qWrite,qRead,iCount):
        bContinueProcessing = False
        if not qRead.empty():
            data = qRead.get().upper()
            #print( "################################## ",data )
            if data=="STOP":
                return True,False #bContinueProcessing
            elif data=="EXCEPTION":
                raise Exception("Software Exception")
            elif data=="READ":
                qWrite.put(self._get_act_data_from_data_nodes(iCount))
            elif data=="READ_EXTENDED":
                qWrite.put(self._get_act_data_from_data_nodes(iCount,bWithStdDev=True))
            elif data=="WRITE":
                data_read = qRead.get()
                #print "==================************** write --> ",data_read
                qWrite.put(self._update_data_for_data_nodes(data_read))
                bContinueProcessing = True    # if we get a write operation --> continue immediately with processing (switch relais)
            elif data=="HISTORY":
                data_read = qRead.get()
                qWrite.put(self._get_history_for_data_node(data_read))
            elif data=="VERSION":
                qWrite.put("Controll Process Version: "+__version__+" from "+__date__)
            else:
                qWrite.put("ERROR: unknown pipe command "+str(data))
        return False,bContinueProcessing

    def _poll_socket_cmd_queue(self,qWrite,qRead,iCount):
        # currently the same implementation as for http server...
        return self._poll_server_cmd_queue(qWrite,qRead,iCount)

    def _wait_and_poll_cmd_queues(self,aDelay,qWrite,qRead,qSocketWrite,qSocketRead,iCount):
        aStartTime = time.time()
        # new since 3.10.2010: stop polling loop and process command if write command is received, see _poll_server_cmd_queue()
        bContinueProcessing = False
        while time.time()-aStartTime<aDelay and not bContinueProcessing:
            #DEBUG: print self._get_act_data_from_data_nodes(iCount)
            # check for queue commands
            #print("poll srv")
            bStop,bContinueProcessing = self._poll_server_cmd_queue(qWrite,qRead,iCount)
            if bStop:
                return True
            # check for socket commands      
            #print("poll socket")
            bStop,bContinueProcessing = self._poll_socket_cmd_queue(qSocketWrite,qSocketRead,iCount)
            if bStop:
                return True
            time.sleep(0.1)
        return False

    def _poll_for_commands(self,qWrite,qRead,qSocketWrite,qSocketRead):
        bContinueProcessing = False
        bStop = False
        bStop,bContinueProcessing = self._poll_server_cmd_queue(qWrite,qRead,self.iCount)
        if bStop:
            return bStop
        bStop,bContinueProcessing = self._poll_socket_cmd_queue(qSocketWrite,qSocketRead,self.iCount)
        return bStop
        
    def _wait(self,aDelay):
        time.sleep(aDelay)
        return False

    def _initialize(self):
        self.sMasterId = "None"
        self._sort_list()
        for e in self:
            e.clock_init()
            
    def _run(self,sFileName,aLock):
        # write description for columns in header line
        aDataHeader = ["tick_no","date","time"]
        for e in self:
            aDataHeader.append(e.get_name())
        sActFileName,aActDate,bNewFile = map_date_to_filename(sFileName,is_debug(),None)
        append_to_file(sActFileName,aDataHeader,sPrefix="#")
        # process the system (endless)
        iCount = get_last_count_from_file(sActFileName)
        bStop = False
        
        while not bStop:
            if is_debug():
                print( "===================================================" )
                print( "TICK",iCount,"TIME",float(iCount)*float(self.DELAY) )
                print( "TICK MASTER_ID value=",self.sMasterId )
            if iCount % 12 == 0:
                # print every minute the timestamp to the consoloe (for debugging)
                print( "ALIVE:", str(datetime.datetime.now()), "; ", end='')
            # Bugfix 12.10.2010: get actual date/time object for determining time/date for output and output file
            # get actual date for next output of data
            sActFileName,aActDate,bNewFile = map_date_to_filename(sFileName,is_debug(),aActDate)
            # prepare item for single step/measurement output
            aLineInfo = [iCount,"%02d.%02d.%04d;%02d:%02d:%02d" % (aActDate.day,aActDate.month,aActDate.year,aActDate.hour,aActDate.minute,aActDate.second)]
            with aLock:
                self.iCount = iCount
                bStop = self.bStop
                for e in self:
                    #try:
                    #if True:
                    val = e._do_clock_tick()
# TODO hier ggf. nicht alle Exceptions fangen, damit ein neustart der Anwendung bei ausfall der Hardware erfolgen kann...                    
                    #except Exception,aException:
                    #    print aException
                    #    sActFileName,aActDate,bNewFile = map_date_to_filename(sFileName,is_debug(),aActDate)
                    #    append_to_file(sActFileName,[str(aException)],sPrefix="#") # log exception as comment line
                    #    val = None
                    _val_for_csv_file = val
                    # Improvement 23.1.2024:
                    # if the value is a container, just use the first element of the container to write into the csv file,
                    # the rest of the content in the container are calculated data used for viewers
                    # i. e. OperatingHoursCounter returns (counter, list(histogram_for_last_days)))
                    if isinstance(_val_for_csv_file, (list, tuple)):
                        _val_for_csv_file = _val_for_csv_file[0] if len(_val_for_csv_file) > 0 else val
                    e._push_value(val)         # set the actual value at the SignalProcessor node (for history of signal)
                    aLineInfo.append(_val_for_csv_file)      # save the actual value for output to file
            # Bugfix 20.8.2010/12.10.2010 --> first append act data into actual csv file, after that update filename with new date for next data output
            # --> fix to remove first line in csv file with old date !
            if bNewFile:
                append_to_file(sActFileName,aDataHeader,sPrefix="#")
            append_to_file(sActFileName,aLineInfo)
            self._wait(self.DELAY)
            iCount += 1
            sys.stdout.flush() 
        return True
        
    def _sort_list(self):
# TODO --> in die richtige Reihenfolge bringen --> gerichete Graphen ?!
        if is_debug():
            print( "WARNING: _sort_list() not implemented yet !" )
    
# *************************************************************************
class SignalProcessor(object):

    MAX_TIMELINE_ELEMENTS = 100
    
    def __init__(self,sName):
        self.sName = sName
        self.aInputSignals = {}
        self.aTimeline = []
        self.bActivated = True
        if g_bUseCache:
            sPersitencePath = add_sdcard_path_if_available(g_sCachePath)
            cacheInfo = pipeprocessing.CreateAveragedHistoryCache(sName, sPersitencePath=sPersitencePath)
            self.aHistoryCache = cacheInfo[0]
            self.fcnValueProducer = cacheInfo[1]
        else:
            self.aHistoryCache = []
            self.fcnValueProducer = None
                
    def finish(self):
        #print "FINISH",self
        pass

    def node_type(self):
        return LAYER_UNKNOWN

    def _get_input_signal(self, input_signal):
        if self.aInputSignals[input_signal] is not None:
            if isinstance(self.aInputSignals[input_signal], str):
                # functools.partial(<bound method TemperatureMeasurement.get_channel of <__main__.TemperatureMeasurement object at 0x000001E41EBDEE40>>, 0)
                return self.aInputSignals[input_signal]
            elif isinstance(self.aInputSignals[input_signal], functools.partial):
                func = self.aInputSignals[input_signal]
                #print("--->",input_signal, self.aInputSignals, input_signal)
                return f"{func.get_node_name()}"
                #return f"{dir(func.func.__func__)} {func.func.__func__} {func.func.__class__}.{func.func.__name__} {func.func.__name__}({func.args})"
            else:
                return str(self.aInputSignals[input_signal].get_name()) #+':'+self.__class__.__name__)
        return 'NO_INPUT_for_'+input_signal

    def get_graph(self):
        sInput = ""
        for input_signal in self.aInputSignals:
            if len(sInput)>0:
                sInput += ", "
            sInput += self._get_input_signal(input_signal)
        return f"{self.sName}:{self.__class__.__name__} <- [{sInput}]"

    def get_dot_data(self):
        """
            Return data which can be used with graphviz (.dot)
        """
        sInput = ""
        sShape = f'shape={_node_type_to_shape(self.node_type())}'
        sColor = f'color={_node_type_to_color(self.node_type())}'
        sInput += self.get_name() #+ '::' + str(self)
        sInput += f'[ {sShape} {sColor} ]\n'
        for input_signal in self.aInputSignals:
            sInput += self._get_input_signal(input_signal)
            sInput += ' -> '
            sInput += self.sName
            sInput += f' [label="input"]'
            sInput += "\n"
        return sInput

    def run_persistence(self):
        self.aHistoryCache.run_persistence()

    def get_name(self):
        return self.sName
        
    def get_additional_data_items(self):
        return None
        
    def is_activated(self):
        return self.bActivated
        
    def set_activated(self,bValue):
        self.bActivated = bValue
        
    def has_input(self):
        return len(self.aInputSignals)>0
        
    def connect_input(self,aSignalNode,sKey):
        self.aInputSignals[sKey] = aSignalNode
        
    def get_input(self,sKey):
        return self.aInputSignals.get(sKey,None)        
        
    def set_value(self,aNewValue):
        """
            Used for manually overriding of values.
        """
        pass
        
    def get_averaged_value(self,noOfSamples=10):
        vals = self.aTimeline[-noOfSamples:]
        count = len(vals)
        if count>0:
            return float(sum(vals))/len(vals)
        else:
            return self.get_value()
            
    def get_stddev_value(self,noOfSamples=10):
        vals = self.aTimeline[-noOfSamples:]
        return stddev(vals)
        
    def get_value(self,iTickBacks=0):
        """
            This ist the output signal of this node.
            
            Returns the actual value (iTickBacks==0)
            or other values back in the time line.
        """
        return self._get_timeline(iTickBacks)
        
    def get_history(self):
        return self.aHistoryCache
        
    def _push_value(self,aValue):
        """
            Will be called from framework only.
        """
        self._push_timeline(aValue)
        if self.fcnValueProducer!=None:
            self.fcnValueProducer(aValue)
        
    def clock_init(self):
        """
            Will be called once before the processing starts.
            Could be used to optimize processing (cache values, etc.).
        """
        pass
        
    def clock_tick(self):
        """
            Will be called for each clock tick in processing mode.
            Must return a value representing the actual signal level.
        """
        return None     # means undefined for this generic node

    def _do_clock_tick(self):
        val = self.clock_tick()
        if is_debug():
            print( "TICK",self.get_name(),"value =",val ) #,"stddev=",self.get_stddev_value() 
        return val

    def _get_timeline(self,iTickBacks=0):
        if abs(iTickBacks)>len(self.aTimeline)-1:	# or iTickBacks<0
            return None
        return self.aTimeline[-1-abs(iTickBacks)]   # return the last element of the timeline
                
    def _push_timeline(self,aValue):
        self.aTimeline.append(aValue)        
        self._verify_timeline()
        
    def _verify_timeline(self):
        if len(self.aTimeline)>self.MAX_TIMELINE_ELEMENTS:
            self.aTimeline = self.aTimeline[len(self.aTimeline)-self.MAX_TIMELINE_ELEMENTS:]

# *************************************************************************
class TemperatureMeasurement(SignalProcessor):

    def __init__(self,sName,aArduino,bNewHeatingcontrolBoardDetected,bTestQualityOfADCs):
        super(TemperatureMeasurement,self).__init__(sName)
        self.aArduino = aArduino
        self.aActValuesT = None
        self.bNewHeatingcontrolBoard = bNewHeatingcontrolBoardDetected
        self.bTestQualityOfADCs = bTestQualityOfADCs
        self.MAX_CHANNELS = 16 # if bNewHeatingcontrolBoardDetected else 12 # since 25.11.2018 the ArduinoMega also supports 16 channels (problem with CONVERTER channel)
        if self.bTestQualityOfADCs:
            self.MAX_CHANNELS = 16+8
        self.iCount = 0             # tick count for simulation modus without arduino hardware

    def node_type(self):
        return LAYER_INPUT

    def finish(self):
        super(TemperatureMeasurement,self).finish()
        if self.aArduino:
            self.aArduino.finish()

# TODO --> wird das ueberhaupt noch benoetigt, da wir nun direkt bei der RS232 Kommunikation abfangen sollte dies hier nie mehr aufgerufen werden ?
    def reset_arduino(self):
        sRs232Name = self.aArduino.sRs232Name if self.aArduino else "???"
        append_to_logfile("RESET_ARDUINO "+sRs232Name,is_debug())
        if self.aArduino:
            self.aArduino.reopen()

    def clock_tick(self):
        ok,sLine = self._read_all(self.MAX_CHANNELS)        # we need only the side effect --> cache object for temperature measurements
        if self.bTestQualityOfADCs and ok:
            self._add_to_quality_data_file("adc_quality.csv",sLine)
        return None

    def get_channel_fcn(self,iChannel):
        ret = functools.partial(self.get_channel,iChannel)
        ret.get_node_name = lambda: f'{self.__class__.__name__}_get_channel_fcn_{iChannel}_'        
        return ret

    def get_channel(self,iChannel):
        return self.aActValuesT[iChannel]

    def _add_to_quality_data_file(self,sFileName,sLine):
        qualityFile = open(sFileName,"a")
        qualityFile.write(sLine)
        qualityFile.close()

    def _try_read(self,sLine):
        iTryCount = 0
        # 10.4.2023: Improvement for Python3 -> check for unkown command and retry last command 
        while (sLine=="" or sLine is None or sLine.startswith("ERROR: unknown command:")) and iTryCount<15:   # org: 3 (until 1.1.2018)
            sLine = self._read_data()
            if iTryCount>0:
                time.sleep(0.2)
            iTryCount += 1
        return sLine
            
    def _read_data(self):
        if self.aArduino:
            #self.aArduino.flushInput()
            #self.aArduino.flushOutput()
            #self.aArduino.reset()
            return self.aArduino.read_temperatures()
        else:
            # for simulation without arduino hardware
            if g_bTest:
                self.iCount += 1
                if self.bNewHeatingcontrolBoard:
                    if self.bTestQualityOfADCs:
                        return "Ch0=25.46;Ch1=65.66;Ch2=7.12;Ch3=28.86;Ch4=41.27;Ch5=38.59;Ch6=14.96;Ch7=81.04;Ch8=20.68;Ch9=-71.00;Ch10=5.32;Ch11=23.12;Ch12=28.86;Ch13=28.86;Ch14=28.86;Ch15=28.86;Ch16=31.12;Ch17=31.12;Ch18=31.12;Ch19=31.12;Ch20=31.12;Ch21=31.12;Ch22=31.12;Ch23=31.12;"+str(self.iCount)
                    else:
                        return "Ch0=25.46;Ch1=65.66;Ch2=7.12;Ch3=28.86;Ch4=41.27;Ch5=38.59;Ch6=14.96;Ch7=81.04;Ch8=20.68;Ch9=-71.00;Ch10=5.32;Ch11=23.23;Ch12=28.86;Ch13=28.86;Ch14=28.86;Ch15=28.86;"+str(self.iCount)
                else:
                    #org: return "Ch0=25.46;Ch1=65.66;Ch2=7.12;Ch3=28.86;Ch4=81.27;Ch5=78.59;Ch6=14.96;Ch7=81.04;Ch8=20.68;Ch9=-71.00;Ch10=1023.00;Ch11=1023.00;"+str(self.iCount)
                    return "Ch0=25.46;Ch1=65.66;Ch2=7.12;Ch3=28.86;Ch4=41.27;Ch5=38.59;Ch6=14.96;Ch7=81.04;Ch8=20.68;Ch9=-71.00;Ch10=5.32;Ch11=23.00;"+str(self.iCount)
            else:
                return ""

    def _read_all(self,iCount):
        # Ch0=25.46;Ch1=65.66;Ch2=17.12;Ch3=28.86;Ch4=81.27;Ch5=78.59;Ch6=14.96;Ch7=81.04;Ch8=20.68;Ch9=-71.00;Ch10=1023.00;Ch11=1023.00;0
        # Ch0=25.74;Ch1=65.66;Ch2=16.95;Ch3=28.85;Ch4=81.29;Ch5=78.56;Ch6=15.04;Ch7=81.00;Ch8=20.81;Ch9=-71.00;Ch10=1023.00;Ch11=1023.00;1
        # Ch0=24.47;Ch1=65.64;Ch2=17.20;Ch3=28.85;Ch4=81.30;Ch5=78.57;Ch6=14.93;Ch7=81.03;Ch8=20.71;Ch9=-71.00;Ch10=1023.00;Ch11=1023.00;2
        sLine = ""
        sLine = self._try_read(sLine)
        if sLine=="":
            self.reset_arduino()
            sLine = self._try_read(sLine)
        try:
            if len(sLine)>0:
                aValues = sLine.split(";")
                self.aActValuesT = [float(aValues[i].split("=")[1]) for i in range(iCount)]
                self.aActValuesT.append(aValues[-1].strip())     # append counter from arduino
                self.aActValuesT.append(len(sLine))      # append length of arduino return string 
                return True,sLine
            else:
                self.aActValuesT = [0 for i in range(iCount)]
                self.aActValuesT.append(0)
                self.aActValuesT.append(0)
                return False,sLine
        except Exception as aExc:
            self.aActValuesT = [0 for i in range(iCount)]
            self.aActValuesT.append(0)
            self.aActValuesT.append(0)
            append_to_logfile("EXCEPTION in _read_all "+str(aExc),is_debug())
            append_to_logfile("LINE=>"+str(sLine)+"< len="+str(len(sLine)),is_debug())
            raise aExc
            return False,sLine       

# *************************************************************************
class Timeline(SignalProcessor):

    def __init__(self,sName):
        super(Timeline,self).__init__(sName)

    def clock_tick(self):
        aNow = datetime.datetime.now()
        return aNow

# *************************************************************************
class Watchdog(SignalProcessor):

    def __init__(self,sName):
        super(Watchdog,self).__init__(sName)
        self.sOriginalName = sName

    def clock_tick(self):
        aNow = datetime.datetime.now()
        aTime = aNow.time()
        # restart heatingconrol and gui on every monday 12:05
        if aNow.weekday()==0 and aTime.hour==12 and aTime.minute==5:
            self.sName = "RESTART"      # WATCHDOG name will be changed in RESTART to signal HTTP Server a restart of the gui
            return "NOW"
        else:
            # reset object name to original one... --> bugfix for crashing QtControlGui
            self.sName = self.sOriginalName
        return None

# *************************************************************************
class RelaisMeasurement(SignalProcessor):

    MAX_CHANNELS = 8

    def __init__(self,sName,aRelaisRS232):
        super(RelaisMeasurement,self).__init__(sName)
        self.aRelaisRS232 = aRelaisRS232

    def node_type(self):
        return LAYER_OUTPUT

# TODO --> wird das ueberhaupt noch benoetigt, da wir nun direkt bei der RS232 Kommunikation abfangen sollte dies hier nie mehr aufgerufen werden ?
    def reset_relais(self):
        sRs232Name = self.aRelaisRS232.sRs232Name if self.aRelaisRS232 else "???"
        append_to_logfile("RESET_RELAIS "+sRs232Name,is_debug())
        if self.aRelaisRS232:
            self.aRelaisRS232.reopen()
        
    # delegate to aRelaisRS232 object 
    def switch_port_fcn(self,iChannel):
        ret_fcn = None
        if self.aRelaisRS232:
            ret_fcn = self.aRelaisRS232.switch_port_fcn(iChannel)
        else:
            # simulate relais object if no hardware is available
            ret_fcn = functools.partial(self.dummy_switch_port,iChannel)
        ret_fcn.get_node_name = lambda: f'{self.__class__.__name__}_switch_port_fcn{iChannel}_'        
        return ret_fcn

    # delegate to aRelaisRS232 object 
    def three_switch_port_fcn(self,iChannelOpen,iChannelClose):
        ret_fcn = None
        if self.aRelaisRS232:
            ret_fcn = self.aRelaisRS232.three_switch_port_fcn(iChannelOpen,iChannelClose)
        else:
            # simulate relais object if no hardware is available
            ret_fcn = functools.partial(self.dummy_three_switch,iChannelOpen,iChannelClose)
        ret_fcn.get_node_name = lambda: f'{self.__class__.__name__}_switch_port_fcn{iChannelOpen}_{iChannelClose}_'        
        return ret_fcn

    def switch_port_for_not_value_fcn(self,iChannel):
        ret_fcn = None
        if self.aRelaisRS232:
            ret_fcn = self.aRelaisRS232.switch_port_for_not_value_fcn(iChannel)
        else:
            # simulate relais object if no hardware is available
            ret_fcn = functools.partial(self.dummy_switch_port_for_not_value,iChannel)
        ret_fcn.get_node_name = lambda: f'{self.__class__.__name__}_switch_port_fcn{iChannel}_'        
        return ret_fcn
        
    def switch_virtual_port_fcn(self,iVirtualChannel):
        ret_fcn = None
        if self.aRelaisRS232:
            ret_fcn = self.aRelaisRS232.switch_virtual_port_fcn(iVirtualChannel)
        else:
            # simulate relais object if no hardware is available
            ret_fcn = functools.partial(self.dummy_virtual_switch_port,iVirtualChannel)
        ret_fcn.get_node_name = lambda: f'{self.__class__.__name__}_switch_port_fcn{iVirtualChannel}_'        
        return ret_fcn

    def dummy_three_switch(self,iChannelOpen,iChannelClose,iValue):
        return False

    def dummy_switch_port(self,iChannel,bValue):
        return False

    def dummy_switch_port_for_not_value(self,iChannel,bValue):
        return False

    def dummy_virtual_switch_port(self,iChannel,bValue):
        return False

    def finish(self):
        super(RelaisMeasurement,self).finish()
        if self.aRelaisRS232:
            self.aRelaisRS232.finish()

    def clock_tick(self):
        ret = None
        if self.aRelaisRS232:
            # same wokaround handling as for arduino
            try:
                ret = self.aRelaisRS232.update_status()
            except Exception as e:
                append_to_logfile(f"ERROR in RelaisMeasurement.clock_tick() exception: {e} time:"+str(datetime.datetime.now()),is_debug())
                self.reset_relais()
                if self.aRelaisRS232:
                    ret = self.aRelaisRS232.update_status()
        return ret
        
# *************************************************************************
class DataSource(SignalProcessor):

    def __init__(self,sName,fcnDataSource=None):
        super(DataSource,self).__init__(sName)
        self.fcnDataSource = fcnDataSource

    def clock_tick(self):
        if self.fcnDataSource!=None:
            return self.fcnDataSource()
        else:
            return None
        
# *************************************************************************
class Esp32HttpTemperature(DataSource):

    def __init__(self,sName):
        super(Esp32HttpTemperature,self).__init__(sName,fcnDataSource=self.read_temperature)
        self.last_value = -1.0

    def read_temperature(self):
        url = "http://192.168.178.50"
        try:
            response = requests.get(url, timeout=3)
            if response.status_code == 200:
                txt = response.text
                TEMPERATURE = "Temperature:"
                pos_start = txt.find(TEMPERATURE)
                pos_stop = txt.find("&deg;")
                if pos_start>=0 and pos_stop>=0:
                    val = float(txt[pos_start+len(TEMPERATURE):pos_stop])
                else:
                    val = 0.0
                self.last_value = val
                return val
        except requests.Timeout:
            pass
        except requests.RequestException as e:
            print("Error in read_temperature() via requests:", e)
        except Exception as e:
            print("Error in read_temperature():", e)
        return self.last_value

# *************************************************************************
class TickCounterSource(DataSource):

    def __init__(self,sName):
        super(TickCounterSource,self).__init__(sName,fcnDataSource=self.update_count)
        self.iCount = 0
                
    def update_count(self):
        self.iCount += 1
        return self.iCount
        
# *************************************************************************
class EnableTimer(SignalProcessor):     # new since 22.10.2010 

    def __init__(self,sName,lstStartStopTimeTuples,returnNoneIfOff=False):
        """
            Tuples could be (Start,Stop) or (Start,Stop,[weekday1,weekday2,...]).
            Item in tuple may be datetime.time or datetime.date or datetime.datetime
        """
        super(EnableTimer,self).__init__(sName)
        self.lstStartStopTimeTuples = lstStartStopTimeTuples
        self.returnNoneIfOff = returnNoneIfOff

    def _get_current_date_intervals(self, current_year):

        # shift time interval in reference year (2010) into time interval of current year, i. e. 2021
        # 11.2009 -> 11.2020
        #  3.2010 ->  3.2021
        #  6.2011 ->  6.2022
        def replace_ref_with_cur_year(ref_date, current_year):
            cur_date = ref_date
            if type(ref_date)==datetime.date:
                cur_date = datetime.date(current_year+(ref_date.year-START_YEAR), ref_date.month, ref_date.day)
            if type(ref_date)==datetime.datetime:
                cur_date = datetime.datetime(current_year+(ref_date.year-START_YEAR), ref_date.month, ref_date.day, ref_date.hour, ref_date.minute, ref_date.second)
            return cur_date

        # replace reference year with current year
        ret = []
        for e in self.lstStartStopTimeTuples:
            aNewStartValue = replace_ref_with_cur_year(e[0], current_year)
            aNewStopValue = replace_ref_with_cur_year(e[1], current_year)
            if len(e)>2:
                data = tuple([aNewStartValue, aNewStopValue] + list(e[2:]))   # add the rest of the tuple to the result, i. e. add the weekday list (like [5,6])
            else:
                data = (aNewStartValue, aNewStopValue)
            ret.append( data )
        return ret

    def get_additional_data_items(self):
        sIntervals = "" #str(self.lstStartStopTimeTuples)
        aNow = datetime.datetime.now()
        for e in self._get_current_date_intervals(aNow.year):
            sDays = ""
            if len(e)>2:
                sDays = str(e[2])
            sIntervals += "start=" + str(e[0]) + " stop=" + str(e[1]) + " " + sDays + ",<br>"
        return {self.get_name()+'_INTERVALS':sIntervals}
        
    def clock_tick(self):
        aNow = datetime.datetime.now()
        aTime = aNow.time()
        aDate = aNow.date()
        aWeekday = aNow.weekday()
        for e in self._get_current_date_intervals(aNow.year):
            aCurrentValue = aTime
            aStartValue = e[0]
            aStopValue = e[1]
            if type(aStartValue)==datetime.date:
                aCurrentValue = aDate
            if type(aStartValue)==datetime.datetime:
                aCurrentValue = aNow
            # if one time/date interval fits --> return 1
            # only time/date interval check
            if len(e)==2 and aCurrentValue>=aStartValue and aCurrentValue<=aStopValue:
                return 1
            # weekday and time/date interval check
            if len(e)>2 and (aWeekday in e[2]) and aCurrentValue>=aStartValue and aCurrentValue<=aStopValue:
                return 1
        # if we reach this point, then we want to return 0 == False -> convert return value to None if requested
        if self.returnNoneIfOff:
            return None
        return 0
 
# *************************************************************************
class AndNode(SignalProcessor):     # new since 4.1.2019 

    def __init__(self,sName,lstNodes,defaultValueForNone = 1):
        super(AndNode,self).__init__(sName)
        self.lstNodes = lstNodes
        self.defaultValueForNone = defaultValueForNone
        # add all nodes to the input signals dictionary
        count = 0
        for node in self.lstNodes:
            count += 1
            self.aInputSignals['node_'+str(count)] = node

    def clock_tick(self):
        """
            And operator for nodes -> true <-> value != 0; False otherwise => returns 0 or 1
            If all operands are None -> returns None
        """
        value = 1
        all_none = True
        for e in self.lstNodes:
            v = e.get_value() #clock_tick()
            if v==None:
                v = self.defaultValueForNone  # do not influence the result if value is None !
            else:
                all_none = False
                v = int(v)
            #value = value and v               # or e.get_value() but this is maybe the last value because framework has not evaluated all nodes yet in this clock tick
            value = 1 if value!=0 and v!=0 else 0
        if all_none:
            return None
        return value       
 
# *************************************************************************
class OrNode(SignalProcessor):     # new since 4.1.2019 

    def __init__(self,sName,lstNodes,defaultValueForNone = 0):
        super(OrNode,self).__init__(sName)
        self.lstNodes = lstNodes
        self.defaultValueForNone = defaultValueForNone
        # add all nodes to the input signals dictionary
        count = 0
        for node in self.lstNodes:
            count += 1
            self.aInputSignals['node_'+str(count)] = node

    def clock_tick(self):
        """
            Or operator for nodes -> true <-> value != 0; False otherwise => returns 0 or 1
            If all operands are None -> returns None
        """
        value = 0
        all_none = True
        for e in self.lstNodes:
            v = e.get_value() #clock_tick()            
            if v==None:
                v = self.defaultValueForNone  # do not influence the result if value is None !
            else:
                all_none = False
                v = int(v)
            #value = value or v                # or e.get_value() but this is maybe the last value because framework has not evaluated all nodes yet in this clock tick
            value = 1 if value!=0 or v!=0 else 0
        if all_none:
            return None
        return value       
 
# *************************************************************************
class NotNode(SignalProcessor):     # new since 4.1.2019 

    def __init__(self,sName,aNode):
        super(NotNode,self).__init__(sName)
        self.aNode = aNode
        self.aInputSignals['not_node'] = self.aNode

    def clock_tick(self):
        value = self.aNode.get_value() #clock_tick()            # or e.get_value() but this is maybe the last value because framework has not evaluated all nodes yet in this clock tick
        if value==None:
            return None
        return 1 if int(value)==0 else 0
 
# *************************************************************************
class CompareToValueNode(SignalProcessor):     # new since 5.10.2024

    def __init__(self,sName,aNode,aValue,aCompareOp = lambda a, b: a == b):
        super(CompareToValueNode,self).__init__(sName)
        self.aNode = aNode
        self.aValue = aValue
        self.aCompareOp = aCompareOp
        self.aInputSignals['compare_to_value_node'] = self.aNode

    def clock_tick(self):
        _current_val = self.aNode.get_value()
        if _current_val is not None:
            value = type(self.aValue)(_current_val) if self.aValue is not None else _current_val
        else:
            value = _current_val
        return 1 if self.aCompareOp(value,self.aValue) else 0
 
# *************************************************************************
class OperatingHoursCounter(SignalProcessor):     # new since 8.1.2024 

    MAX_DAY_HISTOGRAM_LENGTH = 14

    def __init__(self,sName,aNode,init_value=0.0):
        super(OperatingHoursCounter,self).__init__(sName)
        self.aNode = aNode
        self.dOperatingHoursCounter = init_value
        self.lstDayHistogram = []
        self.aLastTickDate = datetime.datetime.now().date()
        self.iCurrentTickCount = 0
        self.iMaxTickCountForSave = int(60.0 / ControlEngine.DELAY) # write file once a minute
        self.dLastTickPerfCounter = None
        self._load_data()
        self.aInputSignals['node'] = self.aNode

    def run_persistence(self):
        self._save_data()

    def clock_tick(self):
        current_tick = time.perf_counter()
        current_tick_date = datetime.datetime.now().date()
        current_value = self.aNode.get_value()
        is_running = int(current_value) == 1 if current_value is not None else False
        if is_running:
            self.dOperatingHoursCounter += ControlEngine.DELAY if self.dLastTickPerfCounter is None else current_tick-self.dLastTickPerfCounter
            self.dLastTickPerfCounter = current_tick
            self.iCurrentTickCount += 1
            if self._check_for_write_data():
                self._save_data()
        else:
            self.dLastTickPerfCounter = None
        if current_tick_date > self.aLastTickDate:
            self._add_to_histogram(self.dOperatingHoursCounter)
        self.aLastTickDate = current_tick_date
        return (self.dOperatingHoursCounter,self.lstDayHistogram)

    def _add_to_histogram(self,value):
        if len(self.lstDayHistogram) >= self.MAX_DAY_HISTOGRAM_LENGTH:
            # if list becomes to large -> remove oldes day value from list -> this is the first element
            del self.lstDayHistogram[0]
        self.lstDayHistogram.append(value)

    def _load_data(self):
        sFileName = self._get_filename()
        if os.path.exists(sFileName):
            ok,data = read_data(sFileName)
            if ok:
                self._set_persistence_data(data)

    def _save_data(self):
        data = self._get_persistence_data()
        write_data(self._get_filename(), data)

    def _set_persistence_data(self,data):
        self.dOperatingHoursCounter = data[0]
        if len(data)>1:
            self.lstDayHistogram = data[1]

    def _get_persistence_data(self):
        return (self.dOperatingHoursCounter,self.lstDayHistogram)

    def _get_filename(self):
        return add_sdcard_path_if_available(g_sPersistencePath+os.sep+self.get_name()+PERSISTENCE_EXTENSION)

    # write the persistent value to file only every 60 seconds, or every 5 minutes...
    def _check_for_write_data(self):
        if self.iCurrentTickCount >= self.iMaxTickCountForSave:
            self.iCurrentTickCount = 0
            return True
        return False

# *************************************************************************
class TemperatureSource(DataSource):

    def __init__(self,sName,fcnDataSource=None,dValueForJumpCheck=None,bCheckForShift=False):
        super(TemperatureSource,self).__init__(sName,fcnDataSource)
        self.dValueForJumpCheck = dValueForJumpCheck
        self.bCheckForShift = bCheckForShift
        self.dLastGoodValue = None
        self.dShift = 0.0
        self.aInputSignals['data_source'] = fcnDataSource
        
    def node_type(self):
        return LAYER_INPUT

#    def clock_tick_for_jump(self):
#        value = super(TemperatureSource,self).clock_tick()              # 18.0
#        if self.dValueForJumpCheck!=None and self.dLastGoodValue!=None: # lastVaue = 8.0
#            delta = value - self.dLastGoodValue                         # delta = 10.0
#            #self.dLastGoodValue = value                                # do not update last value ! so only one jump 
#            if math.fabs(delta)>=self.dValueForJumpCheck:               # jumpCheck = 7.5 
#                value = value - delta                                   # newValue = 18.0 - 10.0 = 8.0
#        else:
#            self.dLastGoodValue = value
#        return value

    def clock_tick(self):
        value = super(TemperatureSource,self).clock_tick()              # switch-down: value < 5.0    # switch-up: value > 12.5
        if self.bCheckForShift:
            #if value<-2.0:
            #    self.dShift = 0.0
            #el
            if value<6.5:
                self.dShift = 0.0
            elif value>12.5:
                self.dShift = 11.0
            value = value - self.dShift
        return value
        
# *************************************************************************
class LightSource(DataSource):

    def __init__(self,sName,fcnDataSource=None):
        super(LightSource,self).__init__(sName,fcnDataSource)
        
# *************************************************************************
class ThermicalSolarCollectorControl(SignalProcessor):        

    INPUT_ID_SOLAR = "solar"
    INPUT_ID_BUFFER = "buffer"
    INPUT_ID_MANUAL_RESET = "reset"     # new since 27.10.2010: if the manual switch is activated --> reset minimum run time !
    
    DELTA_TEMP = 5.0
    MAX_TEMP = 95.0
    MIN_RUN_TIME = 180.0     # in seconds

    def __init__(self,sName,dDelta=DELTA_TEMP):
        super(ThermicalSolarCollectorControl,self).__init__(sName)
        self.dDelta = dDelta
        self.dLastSwitchChange = None
        self.iLastValue = None
        self.aManualInputNode = None

    def node_type(self):
        return LAYER_CONTROL

    def clock_init(self):
        self.aTempSolarNode = self.get_input(self.INPUT_ID_SOLAR)
        self.aTempBufferNode = self.get_input(self.INPUT_ID_BUFFER)
        self.aManualInputNode = self.get_input(self.INPUT_ID_MANUAL_RESET)
    
    def clock_tick(self):    
        # 27.10.2010
        # an activated manual switch (overwrite of the automatic control) must reset the minimum run time
        # because if we disable the manual switch again we want to evaluate the measurments for the control process
        # otherwise the switch may be set to 1 again !
        if self.aManualInputNode!=None and self.aManualInputNode.is_activated():
            self.dLastSwitchChange = None
        # is control enabled --> first time or last switch is at least 60s in the past 
        if self.dLastSwitchChange==None or time.time()-self.dLastSwitchChange>self.MIN_RUN_TIME:
            #if self.aTempSolarNode.get_value()>self.aTempBufferNode.get_value()+self.dDelta:
            dAverage = self.aTempSolarNode.get_averaged_value()
            # switch on: (solar temperature > buffer temperature + 5) AND (buffer temperature < 95 grad)
            dTemp = self.aTempBufferNode.get_value()
            if dAverage>dTemp+self.dDelta and dTemp<self.MAX_TEMP:                
                iNewValue = 1
            # switch off: solar temperature < buffer temperature + 2.5
            elif dAverage<self.aTempBufferNode.get_value()+self.dDelta*0.5:
                iNewValue = 0
            else:
                iNewValue = self.iLastValue
            if self.iLastValue!=iNewValue:
                self.dLastSwitchChange = time.time()
                self.iLastValue = iNewValue
        return self.iLastValue

# *************************************************************************
class HeatingControl(SignalProcessor):

    INPUT_ID_OUTDOOR = "outdoor"

    SWITCH_ON_TEMP = 10.0    # Grad Celsius

    MIN_RUN_TIME = 180.0     # in seconds

    def __init__(self,sName,dSwitchOnTemp=SWITCH_ON_TEMP):
        super(HeatingControl,self).__init__(sName)
        self.dSwitchOnTemp = dSwitchOnTemp
        self.dLastSwitchChange = None
        self.iLastValue = None

    def node_type(self):
        return LAYER_CONTROL

    def get_additional_data_items(self):
        return {self.get_name()+'_SWITCH_ON_TEMP':self.SWITCH_ON_TEMP}
        
    def clock_init(self):
        self.aTempOutdoorNode = self.get_input(self.INPUT_ID_OUTDOOR)
    
    def clock_tick(self):    
        if self.dLastSwitchChange==None or time.time()-self.dLastSwitchChange>self.MIN_RUN_TIME:
            if self.aTempOutdoorNode.get_value()<self.dSwitchOnTemp:
                iNewValue = 1
            else:
                iNewValue = 0
            if self.iLastValue!=iNewValue:
                self.dLastSwitchChange = time.time()
                self.iLastValue = iNewValue
        return self.iLastValue

# *************************************************************************
class DesiredValueForHeatingMixerControl(HeatingControl):            
    
    """
        T > 10 Grad   ==> aus
        T = 10 Grad   ==> 22 Grad 
        T = 0  Grad   ==> MAX Grad
        T < 0  Grad   ==> MAX Grad 
        
        MAX == 28.0
    """
    
    MAX_TEMPERATURE_VALUE = 28.0        # Grad Celsius
    MIN_TEMPERATURE_VALUE = 22.0        # Grad Celsius

    DELTA_TICK_COUNT = 36   # == 60 sec = 12 x 5 sec  # 180 = 36 x 5 sec == 3 min  # new since 9.1.2019

    INPUT_ID_CURRENT_VALUE = "current_value"
    INPUT_ID_ENABLE_MAX_TEMP_CONTROL = "temperature_control"
    
    def __init__(self,sName,dSwitchOnTemp=HeatingControl.SWITCH_ON_TEMP):
        super(DesiredValueForHeatingMixerControl,self).__init__(sName,dSwitchOnTemp)
#        self.m = -0.5  # until 22.11.2010: -0.6    # until 9.11.2010: -0.6666666     # -1.0    # -0.75
#        self.b = 31.5  # until 22.11.2010: 29.5    # until 9.11.2010: 28.5           # 32.0    # 26.5
#        self.m = -0.675  # -0.75   # -0.6666666     # -1.0
#        self.b = 28.0   #26.5   # 28.5           # 32.0
        self.m = -0.5     # new since 5.11.2012
        self.b = self.MAX_TEMPERATURE_VALUE        
        self.dFixedValue = None
        self.aControlTargetTemperature = 36.0  # Grad Celsius
        self.aControlStep = 0.5                # Grad Celsius
        self.aTickCount = 0
        self.aTickCountLastAction = 0
        self._load_data()

    def node_type(self):
        return LAYER_CONTROL

    def run_persistence(self):
        self._save_data()

    def _load_data(self):
        sFileName = self._get_filename()
        if os.path.exists(sFileName):
            ok,data = read_data(sFileName)
            if ok:
                self._set_persistence_data(data)
        
# TODO --> dieser Wert wird haeufig geschrieben --> ggf. Probleme bei Stromausfall !!!        
    def _save_data(self):
        data = self._get_persistence_data()
        write_data(self._get_filename(), data)
        
    def _set_persistence_data(self,data):
        self.b = data[0]
        self.m = data[1]
        self.aControlTargetTemperature = data[2]
        self.aControlStep = data[3]
        
    def _get_persistence_data(self):
        return (self.b,self.m,self.aControlTargetTemperature,self.aControlStep)

    def _get_filename(self):
        return add_sdcard_path_if_available(g_sPersistencePath+os.sep+self.get_name()+PERSISTENCE_EXTENSION)

    def _do_action(self):
        """
            Implement the time constant for control operation.
        """
        self.aTickCount += 1
        # is the time interval for the next control operation expired `
        if self.aTickCount>=self.aTickCountLastAction+self.DELTA_TICK_COUNT:
            self.aTickCountLastAction = self.aTickCount
            # Yes --> do next control operation
            return True
        return False
        
    def _is_control_enabled(self):
        if self.aEnableMaxTempControlSwitch!=None and self.aEnableMaxTempControlSwitch.is_activated():
            return self.aEnableMaxTempControlSwitch.get_value()!=None and int(self.aEnableMaxTempControlSwitch.get_value())>0
        return False

    def _process_control(self):
        if self._is_control_enabled():
            if self._do_action():
                actVal = self.aTemperatureOfBuffer.get_value() if self.aTemperatureOfBuffer else None
                if actVal!=None:
                    if actVal<self.aControlTargetTemperature:
                        newMaxValue = self.get_max_value()-self.aControlStep
                        self.set_max_value(newMaxValue)
                    elif actVal>self.aControlTargetTemperature+0.5:
                        newMaxValue = self.get_max_value()+self.aControlStep
                        self.set_max_value(newMaxValue)
        else:
            # Reset self.b to default value if control is disabled !!!
            self.set_max_value(self.MAX_TEMPERATURE_VALUE)

    def _calc(self,val):
        #if val<-20.0:
        #    return 42.0
        #if val<-4.5:        # new since 14.12.2010      # -2.0:            # new since 4.12.2010
        #    return 31.0     # 30.0
        if val<0.0:          # new since 7.2.2011
            return min(self.MAX_TEMPERATURE_VALUE,self.b)      # we do not reach temperatures above this value
        return round((self.m*val+self.b)*2.0)*0.5       # round to 0.5 --> 0, 0.5, 1, 1.5, 2, ...
            
    def _calc_for_clock_tick(self):
        if self.dFixedValue!=None:
            return self.dFixedValue
        val = self.aTempOutdoorNode.get_value()
        if val<self.dSwitchOnTemp:
            return self._calc(val)
        else:
            return None
            
    def get_additional_data_items(self):
        return { self.get_name()+'_CURRENT_DESIRED_TEMPERATURE':self._calc_for_clock_tick(),
                 self.get_name()+'_CURRENT_MAX_TEMPERATURE':self.b }
        
    def set_max_value(self,newValue):
        if newValue<=self.MAX_TEMPERATURE_VALUE and newValue>=self.MIN_TEMPERATURE_VALUE:
            self.b = newValue
            self._save_data()
            
    def get_max_value(self):
        return self.b
            
    def set_value(self,aNewState):
        if str(aNewState)=="None":
            aNewState = None
        try:
            if aNewState!=None:
                self.dFixedValue = float(aNewState)
            else:
                self.dFixedValue = aNewState # == Non
        except ValueError:
            self.dFixedValue = None
        
    def clock_init(self):
        super(DesiredValueForHeatingMixerControl,self).clock_init()
        self.aTemperatureOfBuffer = self.get_input(self.INPUT_ID_CURRENT_VALUE)
        self.aEnableMaxTempControlSwitch = self.get_input(self.INPUT_ID_ENABLE_MAX_TEMP_CONTROL)
        
    def clock_tick(self):
        self._process_control()
        return self._calc_for_clock_tick()

# *************************************************************************
class HeatingMixerControl(SignalProcessor):            

    INPUT_ID_ACTUAL_VALUE = "actual_value"
    INPUT_ID_DESIRED_VALUE = "desired_value"
    INPUT_ID_CLOSE_SIGNAL = "close_signal"
    INPUT_ID_HEATING_MOTOR = "heating_motor"

    DELTA = 0.5             # minimum derivation between actual and desired value in grad celsius 
    
# TODO --> konstanten als zeitkonstanten angeben --> verwende Konstanten zur Berechnung !    
    DELTA_TICK_COUNT = 36   # == 60 sec = 12 x 5 sec  # new 5.11.2012: 180 sec --> increased timeconstant because of oscillation !

    def __init__(self,sName,actionForMixerClosed = None,queryForMixerIsClosed = None,updateOpenTickCount = None):
        super(HeatingMixerControl,self).__init__(sName)
        self.aActionForMixerClosed = actionForMixerClosed
        self.aQueryForMixerIsClosed = queryForMixerIsClosed
        self.aUpdateOpenTickCount = updateOpenTickCount
        self.bIsMixerClosing = False
        self.aTickCount = 0
        self.aTickCountLastAction = 0

    def node_type(self):
        return LAYER_CONTROL

    def _do_action(self):
        """
            Implement the time constant for control operation.
        """
        self.aTickCount += 1
        # is the time interval for the next control operation expired `
        if self.aTickCount>=self.aTickCountLastAction+self.DELTA_TICK_COUNT:
            self.aTickCountLastAction = self.aTickCount
            # Yes --> do next control operation
            return True
        return False

    def clock_init(self):
        self.aTempActualNode = self.get_input(self.INPUT_ID_ACTUAL_VALUE)
        self.aTempDesiredNode = self.get_input(self.INPUT_ID_DESIRED_VALUE)
        self.aCloseMixerNode = self.get_input(self.INPUT_ID_CLOSE_SIGNAL)
        self.aHeatingMotorNode = self.get_input(self.INPUT_ID_HEATING_MOTOR)
        
    def clock_tick(self):    
        closeMixerSignal = self.aCloseMixerNode.get_value() if self.aCloseMixerNode!=None else None
        heatingMotor = self.aHeatingMotorNode.get_value() if self.aHeatingMotorNode!=None else None
        if closeMixerSignal!=None:
            if closeMixerSignal==1:  # process close mixer signal even if heating motor is not running !
                # check if mixer is already closed ? do nothing if mixer is closed !
                if self.aQueryForMixerIsClosed!=None and self.aQueryForMixerIsClosed():
                    return -1
                self.bIsMixerClosing = True
                return 0                            # close
            # switch state of mixer after mixer is closed
            if self.bIsMixerClosing and closeMixerSignal==0:
                self.bIsMixerClosing = False
                # set mixer state to closed
                if self.aActionForMixerClosed!=None:
                    self.aActionForMixerClosed(True)
        if self._do_action():
            actVal = self.aTempActualNode.get_value()
            desiredVal = self.aTempDesiredNode.get_value()
            if heatingMotor!=None and heatingMotor==0:
                return -1                           # stop --> heating motor is not running --> no closed loop for mixer output
            # no change if any of the control values (act. or desired) is None 
            if actVal!=None and desiredVal!=None:           
                # verify actual value with desired value and make an appropriate action
                if actVal<desiredVal-self.DELTA:
                    if self.aActionForMixerClosed!=None:
                        self.aActionForMixerClosed(False)
                    if self.aUpdateOpenTickCount!=None:
                        self.aUpdateOpenTickCount(1)
                    return +1                       # open
                elif actVal>desiredVal+self.DELTA:
                    if self.aUpdateOpenTickCount!=None:
                        self.aUpdateOpenTickCount(-1)
                    return 0                        # close
        return -1                                   # stop (nothing to do)

# *************************************************************************
class HeatPumpControl(SignalProcessor):

    INPUT_ID_BUFFER = "buffer"
    INPUT_ID_OUTGOING_AIR = "outgoingair"       # == CONVERTER
    INPUT_ID_MANUAL_SWITCH = "manual_switch"
    
# TODO -> nicht einschalten, falls Solar-Pumpe ggf. schon laenger laeuft oder die oberste Temperatur noch ueber einem Schwellwert von xy Grad liegt -> Optimierung fuer dein Einsatz von der Waermepumpe !
    TEMPERATURE_SWITCH_ON  = 37.5           # until 2.12.2010: 36.0
    TEMPERATURE_SWITCH_OFF = 41.0
    TEMPERATURE_OUTGOING_AIR = -0.5 #2.8 # PATCH FOR DEFECT CONVERTER TEMPERATURE SENSOR -0.5         # until 2.12.2010: 0.0          # Vereisungs Schutz
    
    MAX_SAFETY_MEAS_COUNT = 720             # until 2.12.2010: 480              # n Messungen nacheinander unterhalb des Safety-Ausschalt-Limits ==> Waermepumpe wirklich ausschalten !
                                            # until 7.2.2012:  600
# TODO (Optimierung) 7.2.2012: ggf. ein Integral bilden, je nachdem wie viel Temperatur unter der Schwelle liegt, um so schneller ausschalten --> je tiefer Temperaturen, desto schneller vereist Waermepumpe
                                    # 48 == 48*5sec = 4min ! (old: 2min)
  
    MIN_RUN_TIME = 720.0                    # until 2.12.2010: 600.0     # in seconds == 10 minutes !
                                            # until 7.2.2012:  900.0

    def __init__(self,sName):
        super(HeatPumpControl,self).__init__(sName)
        self.iCountMeasBelowSafetySwitchOff = 0
        self.dLastSwitchChange = None
        self.iLastValue = None

    def node_type(self):
        return LAYER_CONTROL

    def get_additional_data_items(self):
        return { self.get_name()+'_SWITCH_ON_TEMP':self.TEMPERATURE_SWITCH_ON,
                 self.get_name()+'_SWITCH_OFF_TEMP':self.TEMPERATURE_SWITCH_OFF,
                 self.get_name()+'_SWITCH_OFF_AIR_TEMP':self.TEMPERATURE_OUTGOING_AIR }
        
    def clock_init(self):
        self.aTempBufferNode = self.get_input(self.INPUT_ID_BUFFER)
        self.aTempOutgoingAirNode = self.get_input(self.INPUT_ID_OUTGOING_AIR)
        self.aManualSwitch = self.get_input(self.INPUT_ID_MANUAL_SWITCH)
        
    def clock_tick(self):
        val = self.aTempBufferNode.get_value()
        valAir = self.aTempOutgoingAirNode.get_value()
        iNewValue = self.iLastValue             # default value: nothing to do, do not change the actual state !
        # check safety switch off
        if valAir<self.TEMPERATURE_OUTGOING_AIR:
            self.iCountMeasBelowSafetySwitchOff += 1
            if self.iCountMeasBelowSafetySwitchOff>self.MAX_SAFETY_MEAS_COUNT: 
                iNewValue = 0
                self.iCountMeasBelowSafetySwitchOff = 0 
                sNow = str(datetime.datetime.now())
                append_to_logfile("ERROR: CONVERTER below 1 grad celsius ! "+sNow,is_debug())
        else:
            # at least one measurement is above the safty limit --> reset counter !
            self.iCountMeasBelowSafetySwitchOff = 0 
            # normal operation...
            # --> first check for manual overwriting
            if self.aManualSwitch!=None and self.aManualSwitch.is_activated():
                if self.aManualSwitch.get_value()!=None and int(self.aManualSwitch.get_value())>0:
                    iNewValue = 1
                else:
                    iNewValue = 0
            # --> than check for the need to switch on/off 
            elif self.dLastSwitchChange==None or time.time()-self.dLastSwitchChange>self.MIN_RUN_TIME:
                if val<=self.TEMPERATURE_SWITCH_ON:
                    iNewValue = 1
                elif val>=self.TEMPERATURE_SWITCH_OFF:
                    iNewValue = 0
#                else:
#                    # nothing to do --> leave the actual value unchanged !
#                    iNewValue = self.iLastValue     # Bugfix 8.11.2010: do not change the state  !(T<=36) and !(T>=40)
#            else:
#                iNewValue = self.iLastValue
        if self.iLastValue!=iNewValue:
            self.dLastSwitchChange = time.time()
            self.iLastValue = iNewValue
        return self.iLastValue

# *************************************************************************
class VentilationControl(SignalProcessor):

    INPUT_ID_HEATPUMP_ON = "heatpumpon"
    INPUT_ID_OUTGOING_AIR = "outgoingair"           # == CONVERTER

    TEMPERATURE_OUTGOING_AIR_MIN = 2.0              # Vereisungs Schutz, until 28.11.2010: 10.0, until 7.2.2012: 4.0             

    def __init__(self,sName):
        super(VentilationControl,self).__init__(sName)

    def node_type(self):
        return LAYER_CONTROL

    def clock_init(self):
        self.aTempOutgoingAirNode = self.get_input(self.INPUT_ID_OUTGOING_AIR)
        self.aHeatPumpOn = self.get_input(self.INPUT_ID_HEATPUMP_ON)
        
    def clock_tick(self):
# TODO --> ggf. hysterese zum Ausschalten, d.h. hoehere Temperatur zum ausschalten
# TODO --> ggf. zweiter Eingang ob Solarkreislauf auf Waermepumpe geschaltet ist
        if (self.aHeatPumpOn.get_value()!=None and self.aHeatPumpOn.get_value()>0) or self.aTempOutgoingAirNode.get_value()<self.TEMPERATURE_OUTGOING_AIR_MIN:     #self.aHeatPumpOn.get_value()>0 or
            return 1
        else:
            return 0

# *************************************************************************
class HeatPumpSolarValveControl(SignalProcessor):

    INPUT_ID_SOLAR_FOR_BUFFER = "solarforbuffer"
    INPUT_ID_COLLECTOR = "collector"
    
    MIN_COLLECTOR_TEMPERATURE = 10.0
     
    def __init__(self,sName):
        super(HeatPumpSolarValveControl,self).__init__(sName)

    def node_type(self):
        return LAYER_CONTROL

    def clock_init(self):
        self.aTempCollectorNode = self.get_input(self.INPUT_ID_COLLECTOR)
        self.aSolarForBufferOnNode = self.get_input(self.INPUT_ID_SOLAR_FOR_BUFFER)
        
    def clock_tick(self):
        if self.aSolarForBufferOnNode==None or int(self.aSolarForBufferOnNode.get_value())==1:
            return 0
        else:
            return self.aTempCollectorNode.get_input()>=self.MIN_COLLECTOR_TEMPERATURE 

# *************************************************************************
class ManualSwitch(SignalProcessor):
    
    def __init__(self,sName,aInitState=None):
        super(ManualSwitch,self).__init__(sName)
        self.set_activated(False)
        self.aState = aInitState
        self._load_data()
        verify_path(self._get_filename())

    def node_type(self):
        return LAYER_MANUAL_SWITCH

    def set_value(self,aNewState):
        if str(aNewState)=="None":
            aNewState = None
        self.aState = aNewState
        self.set_activated(aNewState!=None)
        self._save_data()

    def clock_tick(self):    
        if self.is_activated():
            return self.aState
        else:
            return None
                    
    def run_persistence(self):
        self._save_data()

    def _load_data(self):
        sFileName = self._get_filename()
        if os.path.exists(sFileName):
            ok,data = read_data(sFileName)
            if ok:
                self._set_persistence_data(data)
        
    def _save_data(self):
        data = self._get_persistence_data()
        write_data(self._get_filename(), data)
        
    def _set_persistence_data(self,data):
        self.aState = data[0]
        self.set_activated(data[1])
        
    def _get_persistence_data(self):
        return (self.aState,self.is_activated())

    def _get_filename(self):
        return add_sdcard_path_if_available(g_sPersistencePath+os.sep+self.get_name()+PERSISTENCE_EXTENSION)

# *************************************************************************
class ManualSwitchAutoReset(SignalProcessor):
    
    """
        Special class to handle a switch with pulse functionality:
        
        -1 == close, 0 == stop, 1 == open, 2 == open pulse, 3 == close pulse
        
    """
    
    def __init__(self,sName,aInitState=None):
        super(ManualSwitchAutoReset,self).__init__(sName)
        self.set_activated(False)
        self.aState = aInitState
        self.aLastState = aInitState

    def set_value(self,aNewState):
        if str(aNewState)=="None":
            aNewState = None
        self.aLastState = self.aState
        self.aState = aNewState
        self.set_activated(aNewState!=None)

    def clock_tick(self):    
        if self.is_activated():
            ret = self.aState
            # auto reset the values greater than 1
            if ret!=None and int(ret)>=2:
                self.aState = self.aLastState
            return ret
        else:
            return None
        
# *************************************************************************
#class SwitchValue(ManualSwitch):
#    
#    def __init__(self,sName,aInputForOn,aInputForOff,aInitState=None):
#        super(SwitchValue,self).__init__(sName,aInitState)
#        self.aInputForOn = aInputForOn
#        self.aInputForOff = aInputForOff
#
#    def clock_tick(self):    
#        val = self.is_activated()
#        print "----> ",self,val
#        if val:
#            return self.aInputForOn.clock_tick()
#        else:
#            return self.aInputForOff.clock_tick()
#        #if val:
#        #    return str(val)+"/"+str(self.aInputForOn.clock_tick())
#        #else:
#        #    return str(val)+"/"+str(self.aInputForOff.clock_tick())
                            
# *************************************************************************
class ValueSwitch(SignalProcessor):
    
    def __init__(self,sName,aInputForOn,aInputForOff,aManualSwitch):
        super(ValueSwitch,self).__init__(sName)
        self.aInputForOn = aInputForOn          # ManualSwitch ON
        self.aInputForOff = aInputForOff        # ManualSwitch OFF or None
        self.aManualSwitch = aManualSwitch
        self.aInputSignals['input_for_on'] = self.aInputForOn
        self.aInputSignals['input_for_off'] = self.aInputForOff
        self.aInputSignals['input_for_manual_switch'] = self.aManualSwitch

    def clock_tick(self):    
        val = self.aManualSwitch.clock_tick()
        if val==None or int(val)==0:
            return self.aInputForOff.clock_tick()
        else:
            return self.aInputForOn.clock_tick()

# *************************************************************************
class SwitchRelais(SignalProcessor):
    
    """
        Represents a relais which could be switched
        automatically and manually.
        (manual value overrides automatic value)
        Two states possible: "on" and "off"
    """
    
    INPUT_ID = "input"
    INPUT_ID_MANUAL = "manual"
    ENABLE_ID = "enable"
    
    def __init__(self,sName,fcnSignalSideEffect=None,fcnSignalSideEffectForNotEnabled=None):
        super(SwitchRelais,self).__init__(sName)
        # side effects are something like outputs
        self.fcnSignalSideEffect = fcnSignalSideEffect
        self.fcnSignalSideEffectForNotEnabled = fcnSignalSideEffectForNotEnabled
        self.aInputSignals['data_source'] = fcnSignalSideEffect

    def node_type(self):
        return LAYER_OUTPUT

    def get_graph(self):
        sInput = ""
        for input_signal in self.aInputSignals:
            if len(sInput)>0:
                sInput += ", "
            sInput += self._get_input_signal(input_signal)
        return f"{self.sName}:{self.__class__.__name__} <- [{sInput}]"

    # def get_dot_data(self):
    #     """
    #         Return data which can be used with graphviz (.dot)
    #     """
    #     sInput = ""
    #     for input_signal in self.aInputSignals:
    #         sInput += self._get_input_signal(input_signal)
    #         sInput += " -> "
    #         sInput += self.sName
    #         sInput += ' [label="input"]'
    #         sInput += "\n"
    #     return sInput

    def clock_init(self):
        self.aInputNode = self.get_input(self.INPUT_ID)
        self.aManualInputNode = self.get_input(self.INPUT_ID_MANUAL)
        self.aEnableInputNode = self.get_input(self.ENABLE_ID)
        
    def clock_tick(self):        
        ret = None
        if self.aManualInputNode!=None and self.aManualInputNode.is_activated() and self.aManualInputNode.get_value() is not None:
            if int(self.aManualInputNode.get_value())>0:
                ret = 1
            else:
                ret = 0
            #print "MANUAL ENABLED",self.sName,ret
        else:
            if self.aEnableInputNode==None or self.aEnableInputNode.get_value()==1:
                if self.aInputNode!=None and self.aInputNode.get_value()!=None:
                    if int(self.aInputNode.get_value())>0:
                        ret = 1
                    else:
                        ret = 0
            else:
                ret = 0
            #print "AUTOMATIC ENABLED !!!",self.sName,ret
        # process the relais switch as side effect function...
        bEnable = ret!=None and int(ret)==1
        if ret!=None and self.fcnSignalSideEffect:
            self.fcnSignalSideEffect(bEnable)
        #old: if ret!=None and self.fcnSignalSideEffectForNotEnabled:
# TODO --> only send closed signal for Bypass if not already closed !        
        if self.fcnSignalSideEffectForNotEnabled:
            self.fcnSignalSideEffectForNotEnabled(bEnable)  # forward original value, because will be inverted in function !
        return ret

# *************************************************************************
class ThreeStateSwitchRelais(SignalProcessor):
    
    """
        Represents a double relais which could be switched
        automatically and manually.
        (manual value overrides automatic value)
        Three states possible: "open", "close" and "stop"
    """

    INPUT_ID = "input"
    INPUT_ID_MANUAL = "manual"
    ENABLE_ID = "enable"
    
    def __init__(self,sName,fcnSignalSideEffect=None):
        super(ThreeStateSwitchRelais,self).__init__(sName)
        self.fcnSignalSideEffect = fcnSignalSideEffect
        self.bIsClosed = False
        self.iOpenCount = None  # count the ticks for which the mixer is opened
        self._load_data()

    def node_type(self):
        return LAYER_OUTPUT

    def run_persistence(self):
        self._save_data()

    def _load_data(self):
        sFileName = self._get_filename()
        if os.path.exists(sFileName):
            ok,data = read_data(sFileName)
            if ok:
                self._set_persistence_data(data)
        
    def _save_data(self):
        data = self._get_persistence_data()
        write_data(self._get_filename(), data)
        
    def _set_persistence_data(self,data):
        self.bIsClosed = data[0]
        self.iOpenCount = data[1]
        
    def _get_persistence_data(self):
        return (self.bIsClosed,self.iOpenCount)

    def _get_filename(self):
        return add_sdcard_path_if_available(g_sPersistencePath+os.sep+self.get_name()+PERSISTENCE_EXTENSION)
        
    def get_additional_data_items(self):
        return {self.get_name()+'_CURRENT_OPEN_COUNT':self.get_opent_tick_count()}
        
    def is_closed(self):
        return self.bIsClosed
        
    def set_closed(self,val):
        self.bIsClosed = val
        # reset open tick count if mixer is closed
        if val:
            self.iOpenCount = 0
        self._save_data()
        
    def update_open_tick_count(self,count):
        if self.iOpenCount==None:
            self.iOpenCount = 0
        self.iOpenCount += count
        self._save_data()
        
    def get_opent_tick_count(self):
        return self.iOpenCount

    def clock_init(self):
        self.aInputNode = self.get_input(self.INPUT_ID)
        self.aManualInputNode = self.get_input(self.INPUT_ID_MANUAL)
        self.aEnableInputNode = self.get_input(self.ENABLE_ID)
        
    def clock_tick(self):        
        ret = None
        if self.aManualInputNode!=None and self.aManualInputNode.is_activated():
            val = self.aManualInputNode.get_value()
            ret = int(val) if val!=None else None
        else:
            if self.aEnableInputNode==None or self.aEnableInputNode.get_value()==1:
                val = self.aInputNode.get_value()
                ret = int(val) if val!=None else None
            else:
                ret = -1        # means stop !
        # process the relais switch as side effect function...
        # open/close relais for at least one tick == 5sec
        if ret!=None and self.fcnSignalSideEffect:
            self.fcnSignalSideEffect(int(ret))
        return ret        
        
# *************************************************************************
# *************************************************************************
# *************************************************************************
#
#  SOLAR_KVLF + SOLAR_SLVF      -->  ThermicalSolarCollectorControl + ManualSwitch      -->  SwitchRelais
#  (Kollektor)  (Puffer Unten)
#
#  OUTDOOR                      -->  HeatingControl + ManualSwitch                      -->  SwitchRelais
#
#  OUTDOOR                      -->  DesiredValueHeatingControl                         -->  DESIRED_VAL
#
#  MIXER_HEATING + DESIRED_VAL  -->  HeatingMixerControl + ManualSwitch + ManualSwitch  -->  SwitchRelais (open) + SwitchRelais (close)
#
#

#TICK 2 TIME 10.0
#TICK TEMP_MEAS value= None
#TICK LIGHT1 value= 0
#TICK LIGHT2 value= 0
#TICK SOLAR_KVLF value= 0
#TICK SOLAR_SLVF value= 0
#TICK OUTDOOR value= 0
#TICK MIXER_HEATING value= 0
#TICK BUFFER1 value= 0
#TICK BUFFER2 value= 0
#TICK HEATPUMP_OUTGOING_AIR value= 0
#TICK SOLAR_CONTROL value= 0
#TICK HEATING_CONTROL value= 1
#TICK DESIRED_VALUE_MIXER_HEATING_CONTROL value= 0
#TICK MIXER_HEATING_CONTROL value= 0
#TICK HEATPUMP_CONTROL value= 0
#TICK VENTILATION_CONTROL value= 1
#TICK HEATPUMP_SOLAR_VALVE_CONTROL value= 0
#TICK MANUAL_SWITCH_MOTOR_SOLAR value= None
#TICK MANUAL_SWITCH_MOTOR_HEATING value= None
#TICK MANUAL_SWITCH_OPEN_MIXER_HEATING value= None
#TICK MANUAL_SWITCH_CLOSE_MIXER_HEATING value= None
#TICK MANUAL_SWITCH_HEATPUMP value= None
#TICK MANUAL_SWITCH_VENTILATION value= None
#TICK MANUAL_SWITCH_HEATPUMP_SOLAR_VALVE value= None
#TICK SWITCH_MOTOR_SOLAR value= 0
#TICK SWITCH_MOTOR_HEATING value= 1
#TICK SWITCH_OPEN_MIXER_HEATING value= 0
#TICK SWITCH_CLOSE_MIXER_HEATING value= 0
#TICK SWITCH_HEATPUMP value= 0
#TICK SWITCH_VENTILATION value= 1
#TICK SWITCH_HEATPUMP_SOLAR_VALVE value= 0
#TICK RELAIS_MEAS value= None

# temperature objects
SOLAR_KVLF                          = "SOLAR_KVLF"
SOLAR_SLVF                          = "SOLAR_SLVF"
OUTDOOR                             = "OUTDOOR"
MIXER_HEATING                       = "MIXER_HEATING"
BUFFER1                             = "BUFFER1"
BUFFER2                             = "BUFFER2"
HEATPUMP_OUTGOING_AIR               = "OUTGOING_AIR"            # until 4.11.2010: "HEATPUMP_OUTGOING_AIR"
HEAT_CREATOR                        = "HEAT_CREATOR"
WARM_WATER                          = "WARM_WATER"
INGOING_AIR                         = "INGOING_AIR"
CONVERTER                           = "CONVERTER"
ROOM                                = "ROOM"                    # new since 25.11.2018
TESTSENSOR_PT1000                   = "TESTSENSOR_PT1000"       # new since 1.1.2018
ESP32_HTTP_TEMP                     = "ESP32_HTTP_TEMP"         # new since 9.11.2024
# other objects from temperature manager
ARDURINO_COUNT                      = "ARDURINO_COUNT"
TIMELINE                            = "TIMELINE"                # new since 5.1.2019

# control objects
SOLAR_CONTROL                       = "SOLAR_CONTROL"
HEATING_CONTROL                     = "HEATING_CONTROL"
MIXER_HEATING_CONTROL               = "MIXER_HEATING_CONTROL"
HEATPUMP_CONTROL                    = "HEATPUMP_CONTROL"
VENTILATION_CONTROL                 = "VENTILATION_CONTROL"
HEATPUMP_SOLAR_VALVE_CONTROL        = "HEATPUMP_SOLAR_VALVE_CONTROL"

DESIRED_VALUE_MIXER_HEATING_CONTROL = "MIXER_HEATING_DES"     # until 4.11.2010: "DESIRED_VALUE_MIXER_HEATING_CONTROL"

# timers
ENABLE_HEATING_MOTOR                = "ENABLE_HEATING_MOTOR"
ENABLE_HEATING_MOTOR_NIGHT          = "ENABLE_HEATING_MOTOR_NIGHT"
ENABLE_HEATING_MOTOR_VALUE          = "ENABLE_HEATING_MOTOR_VALUE"
ENABLE_HEATING_MOTOR_DATE           = "ENABLE_HEATING_MOTOR_DATE"
ENABLE_HEATING_MOTOR_DATE_AND_VALUE = "ENABLE_HEATING_MOTOR_DATE_AND_VALUE"
ENABLE_MIXER_MAX_TEMP_CONTROL       = "ENABLE_MIXER_MAX_TEMP_CONTROL"
ENABLE_HEATING_MOTOR_ALL_INPUTS     = "ENABLE_HEATING_MOTOR_ALL_INPUTS"
ENABLE_MIXER_MOVEMENT               = "ENABLE_MIXER_MOVEMENT"
ENABLE_HEAT_PUMP                    = "ENABLE_HEAT_PUMP"
ENABLE_HEAT_PUMP_FOR_PV             = "ENABLE_HEAT_PUMP_FOR_PV"
ENABLE_MIXER_CLOSE                  = "ENABLE_MIXER_CLOSE"
ENABLE_DATE_HEATING_MOTOR           = "ENABLE_DATE_HEATING_MOTOR"
ENABLE_ANTI_FIXING_SWITCH           = "ENABLE_ANTI_FIXING_SWITCH"

# manual switches
MANUAL_SWITCH_MOTOR_SOLAR           = "MANUAL_SWITCH_MOTOR_SOLAR"
MANUAL_SWITCH_MOTOR_HEATING         = "MANUAL_SWITCH_MOTOR_HEATING"
MANUAL_SWITCH_HEATING_ONLY_NIGHT    = "MANUAL_SWITCH_HEATING_ONLY_NIGHT"
MANUAL_SWITCH_HEATING_DATE          = "MANUAL_SWITCH_HEATING_DATE"
MANUAL_SWITCH_OPEN_MIXER_HEATING    = "MANUAL_SWITCH_OPEN_MIXER_HEATING"
MANUAL_SWITCH_CLOSE_MIXER_HEATING   = "MANUAL_SWITCH_CLOSE_MIXER_HEATING"
MANUAL_SWITCH_MIXER_HEATING         = "MANUAL_SWITCH_MIXER_HEATING"
MANUAL_SWITCH_HEATPUMP              = "MANUAL_SWITCH_HEATPUMP"
MANUAL_SWITCH_VENTILATION           = "MANUAL_SWITCH_VENTILATION"
MANUAL_SWITCH_HEATPUMP_SOLAR_VALVE  = "MANUAL_SWITCH_HEATPUMP_SOLAR_VALVE"
MANUAL_SWITCH_BOOSTER               = "MANUAL_SWITCH_BOOSTER"
MANUAL_SWITCH_MAX_TEMP_CONTROL      = "MANUAL_SWITCH_MAX_TEMP_CONTROL"
MANUAL_SWITCH_HEATPUMP_ONLY_WITH_PV = "MANUAL_SWITCH_HEATPUMP_ONLY_WITH_PV"

# switches
SWITCH_MOTOR_SOLAR                  = "SWITCH_MOTOR_SOLAR"              # Relais 1
SWITCH_MOTOR_HEATING                = "SWITCH_MOTOR_HEATING"            # Relais 2
SWITCH_MIXER_HEATING                = "SWITCH_MIXER_HEATING"            # Relais 7, 8
SWITCH_HEATPUMP                     = "SWITCH_HEATPUMP"                 # Relais 3
SWITCH_VENTILATION                  = "SWITCH_VENTILATION"              # Relais 4, 5 -> Bypass Lueftungsanalge
SWITCH_HEATPUMP_SOLAR_VALVE         = "SWITCH_HEATPUMP_SOLAR_VALVE"         # not used any more -> mapped to virtual port -> channel used to control Bypass Lueftungsanalge
SWITCH_BOOSTER                      = "SWITCH_BOOSTER"                  # Relais 6    -> neu: Zirkulationspumpe fuer Waermepumpe

# operating hours counters
OPERATING_HOURS_HEATPUMP            = "OPERATING_HOURS_HEATPUMP"
OPERATING_HOURS_MOTOR_HEATING       = "OPERATING_HOURS_MOTOR_HEATING"
OPERATING_HOURS_MOTOR_SOLAR         = "OPERATING_HOURS_MOTOR_SOLAR"

# logical operations
LOGIC_IS_MANUAL_SWITCH_HEATPUMP_ONLY_WITH_PV        = "LOGIC_IS_MANUAL_SWITCH_HEATPUMP_ONLY_WITH_PV"
LOGIC_IS_NOT_MANUAL_SWITCH_HEATPUMP_ONLY_WITH_PV    = "LOGIC_IS_NOT_MANUAL_SWITCH_HEATPUMP_ONLY_WITH_PV"
LOGIC_ENABLE_HEATPUMP_VIA_PV                        = "LOGIC_ENABLE_HEATPUMP_VIA_PV"

# others
TICK_COUNTER                        = "TICK_COUNTER"
WATCHDOG                            = "WATCHDOG"

#
# Temperatur Sensoren:
# ADC-Nr                    Nr im Plot                  Sensor im Testbetrieb
#----------------------------------------------------------------------------------------
#   0   SolarCollector         2 SOLAR_KVLF             PT1000 (Conrad, klein)
#   1   Buffer 3               3 SOLAR_SLVF             NTC20 (Haus-Sensor)
#   2   Outdoor                7 OUTDOOR                NTC20 (Conrad, klein)
#   3   Mixer                 10 MIXER_HEATING          NTC20 (Conrad, klein)
#   4   Buffer 1               5 BUFFER1                NTC20 (Conrad, klein)
#   5   Buffer 2               6 BUFFER2                NTC20 (Conrad, klein)
#   6   Outgoing Air           8 HEATPUMP_OUTGOING_AIR  NTC20 (Conrad, klein) (in der Abluft von Lueftungsanlage zur Waermepumpe)
#   7   Warmwasser             4 WARM_WATER             NTC20 (Conrad, klein)    
#   8   Waermeerzeuger         1 HEAT_CREATOR           NTC20 (Conrad, klein) (im Zulauf von Waermepumpe zu Buffer)
#   9   Ingoing Air              INGOING_AIR            NTC20 (Conrad, klein) <not connected yet>       ==> PT1000 bei ArduinoMega Board 
#  10   Converter              9 CONVERTER              NTC20 (Conrad, klein) org: <not used>   LIGHT1
#  11   ---                                             NTC20 (Conrad, klein) <not used>        LIGHT2  ==> Special Handling bei ArduinoMega Board (kann nicht für NTC20 genutzt werden)
#  12   Room                     ROOM                   NTC20                                           ==> ROOM since 25.11.2018
#  ..
#  15   PT1000 Test Sensor    11 TESTSENSOR_PT1000      PT1000 (Haus-Sensor)
#
# Relais:
#   0   Solar Motor
#   1   Heating Motor
#   2   Waermepumpe                                    --> Ausgang #3
#   3,4 Lueftungsanlage & Bypass                                                                                                                      ==> ACHTUNG: immer wechselseitig schalten !! Aenhnlich wie Mixer
#   4,3 (Ventil Waermepumpe / Solarbetrieb) / Bypass   --> Ausgang #5  ==> seit 8.9.2019: Bypass Kanal Close (Braun), #3 ist Bypass Kanal Open (Blau) ==> ACHTUNG: immer wechselseitig schalten !! Aenhnlich wie Mixer
#   5   (Booster) / Warmwasser Zirkulationspumpe                       ==> seit August 2013: Zirkulationspumpe fuer Waermepumpe !!! Muss laufen wenn Waermepumpe laeuft !!!
#   6,7 Mixer 
# 

def detect_control_hardware(aHeatingControlBoard):
    """
        Try to communicate with control board hardware via RS232,
        try to read board version.
        Returns:
         - True -> for new Heating control board
         - False -> for Arduino/Conrad control board(s)
    """
    bNewHeatingcontrolBoardDetected = True
    while True:
        with ResouceScope(aHeatingControlBoard,g_bExclusiveRs232) as device:
            device.write("VERSION;")
            cmdResult = device.readline() # expected: VERSION=2.0 from 12.12.2017  or  VERSION=1.3 from 6.11.2010
            append_to_logfile("--> VERSION: "+str(cmdResult),is_debug())
            print("Try to read VERSION:", str(cmdResult), "length=", len(cmdResult), "Device:", aHeatingControlBoard.sRs232Name)
            # 4.2023:after booting Raspberry Pi the first communication via RS232 does not work correctly...
            # see ASCII Codes: https://johndecember.com/html/spec/ascii.html

            if cmdResult.startswith("VERSION="):
                sVersion = cmdResult[8:11]
                if sVersion.startswith("1."):
                    bNewHeatingcontrolBoardDetected = False
                return bNewHeatingcontrolBoardDetected
        
            # if we had no success -> try again until success!
            time.sleep(5)

def configure_control():
    if is_debug():
        print( "configuring control" )
    sNow = str(datetime.datetime.now())
    aLstDevices = find_all_usb_rs232_devices()
    #print "###### all rs232",aLstDevices,sNow
    append_to_logfile("###### all rs232: "+str(aLstDevices)+" "+sNow,is_debug())
    # try first the default com port --> use UART !
    
    bNewHeatingcontrolBoardDetected = True
    if g_bTest:
        bNewHeatingcontrolBoardDetected = True
        aHeatingControlBoard = None
    else:
        # new HeatingControlBoard (firmware >= 2.0) or old combination ArduinoMega & ConradMultiRelais (firmware < 2.0)
        aHeatingControlBoard = HeatingControlBoard(DEVICE_RS232_HEATINGCONTROLBOARD,baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=5.0)
        bNewHeatingcontrolBoardDetected = detect_control_hardware(aHeatingControlBoard) 
    append_to_logfile("--> HeatingControlBoard: "+str(aHeatingControlBoard),is_debug())

    if False:
        with ResouceScope(aHeatingControlBoard,g_bExclusiveRs232) as device:
            device.write("VERSION;")
            cmdResult = device.readline() # expected: VERSION=2.0 from 12.12.2017  or  VERSION=1.3 from 6.11.2010
            append_to_logfile("--> VERSION: "+str(cmdResult),is_debug())
            # after booting Raspberry Pi the first communication via RS232 does not work correctly...
            # see: https://johndecember.com/html/spec/ascii.html
            if cmdResult.startswith("ERROR: unknown command: ;"):
                device.write("VERSION;")
                cmdResult = device.readline() # expected: VERSION=2.0 from 12.12.2017  or  VERSION=1.3 from 6.11.2010
                append_to_logfile("--> second try VERSION: "+str(cmdResult),is_debug())        
            if cmdResult.startswith("VERSION="):
                sVersion = cmdResult[8:11]
                if sVersion.startswith("1."):
                    bNewHeatingcontrolBoardDetected = False            
            else:
                # something went wrong... (or test modus)
                append_to_logfile("WARNING --> HeatingControlBoard: None",is_debug())
                aHeatingControlBoard = None
        
    if aHeatingControlBoard and bNewHeatingcontrolBoardDetected:
        with ResouceScope(aHeatingControlBoard,g_bExclusiveRs232) as device:
            device.write("RESTARTS;")
            cmdResult = device.readline()
            append_to_logfile("--> Number of restarts: "+cmdResult,is_debug())
        
    append_to_logfile("--> bNewHeatingcontrolBoardDetected: "+str(bNewHeatingcontrolBoardDetected),is_debug())
    if bNewHeatingcontrolBoardDetected:
        aArduino = aHeatingControlBoard
        aRelaisManager = aHeatingControlBoard
    else:
        aHeatingControlBoard.close()
        aHeatingControlBoard = None
        
        if g_bTest:
            aRelaisManager = None
        else:
            aRelaisManager = ConradMultiRelais(DEVICE_RS232_RELAIS,baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=2.0)
            if not aRelaisManager.is_ok() or not aRelaisManager.ping():
                aRelaisManager.close()
                aLstDevices,aRelaisManager = find_rs232_device_for_hardware_class(aLstDevices,ConradMultiRelais,baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=5.0)  # old: timeout=0.5
        # try first the default com port --> use UART !
        if g_bTest:
            aArduino = None
        else:
            aArduino = ArduinoMega(DEVICE_RS232_ARDUINO,baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=5.0)
            if not aArduino.is_ok() or not aArduino.ping():
                aArduino.close()
                aLstDevices,aArduino = find_rs232_device_for_hardware_class(aLstDevices,ArduinoMega,baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=5.0)
    
    append_to_logfile("--> TemperatureMeasurement: "+str(aArduino),is_debug())
    append_to_logfile("--> RelaisMeasurement: "+str(aRelaisManager),is_debug())
    aTempMeasurement = TemperatureMeasurement("TEMP_MEAS",aArduino,bNewHeatingcontrolBoardDetected,bTestQualityOfADCs=False)    # proxy for Ardurino, to synchronize update of temperatures in processing engine/clock
    aRelaisMeasurement = RelaisMeasurement("RELAIS_MEAS",aRelaisManager)   # Output
    
    aLightSensor1 = LightSource("LIGHT1",aTempMeasurement.get_channel_fcn(10))
    aLightSensor2 = LightSource("LIGHT2",aTempMeasurement.get_channel_fcn(11))
    
    aTimeline = Timeline(TIMELINE)

    aEsp32HttpTemperature = Esp32HttpTemperature(ESP32_HTTP_TEMP)

    # Solar Kollektor Regelung
    #   Solar Pumpe
    aTempSolar = TemperatureSource(SOLAR_KVLF,aTempMeasurement.get_channel_fcn(0))
    aTempSolarBuffer = TemperatureSource(SOLAR_SLVF,aTempMeasurement.get_channel_fcn(1))
    aSolarControl = ThermicalSolarCollectorControl(SOLAR_CONTROL)     
    aManualSwitchSolarMotor = ManualSwitch(MANUAL_SWITCH_MOTOR_SOLAR)
    aSolarMotor = SwitchRelais(SWITCH_MOTOR_SOLAR,aRelaisMeasurement.switch_port_fcn(0))                       # Pumpe fuer Solarkreislauf
    aOperatingHoursMotorSolar = OperatingHoursCounter(OPERATING_HOURS_MOTOR_SOLAR,aSolarMotor)
    aSolarControl.connect_input(aTempSolar,aSolarControl.INPUT_ID_SOLAR)
    aSolarControl.connect_input(aTempSolarBuffer,aSolarControl.INPUT_ID_BUFFER)
    aSolarControl.connect_input(aManualSwitchSolarMotor,aSolarControl.INPUT_ID_MANUAL_RESET)
    aSolarMotor.connect_input(aSolarControl,aSolarMotor.INPUT_ID)
    aSolarMotor.connect_input(aManualSwitchSolarMotor,aSolarMotor.INPUT_ID_MANUAL);

    # Heizung Regelung
    #   Heizung Pumpe
    aTempOutdoor =  TemperatureSource(OUTDOOR,aTempMeasurement.get_channel_fcn(2))
    aHeatingControl = HeatingControl(HEATING_CONTROL)
    aHeatingControl.connect_input(aTempOutdoor,aHeatingControl.INPUT_ID_OUTDOOR)
    aManualSwitchHeatingMotor = ManualSwitch(MANUAL_SWITCH_MOTOR_HEATING)
    # 3h + 4h == 7h Heizen bis 9.12.2010
# TODO --> anscheinend wird die History der Messwerte nicht protokolliert/aktualisiert im cache, wenn geheizt wird !!!
# TODO --> Mixer-Regelung laeuft anscheinend in aktueller Version nicht !!!
    START_DISABLE_TIME1_HEATPUMP_HOUR = 11
    START_DISABLE_TIME1_HEATPUMP_MINUTE = 29
    END_DISABLE_TIME1_HEATPUMP_HOUR = 13
    END_DISABLE_TIME1_HEATPUMP_MINUTE = 2
    START_DISABLE_TIME2_HEATPUMP_HOUR = 17
    START_DISABLE_TIME2_HEATPUMP_MINUTE = 29
    END_DISABLE_TIME2_HEATPUMP_HOUR = 19
    END_DISABLE_TIME2_HEATPUMP_MINUTE = 2
    STOP_HEATING_MOTOR_TIME1 = 14
    STOP_HEATING_MOTOR_TIME2 = 17
    STOP_HEATING_MOTOR_TIME3 = 3
    # *** Heatingpump connected to Waermepumenstrom Zaehler -> Ausschaltzeiten zu Stosszeiten ***
    # aEnableTimer = EnableTimer(ENABLE_HEATING_MOTOR,[(datetime.time(8,00,0),datetime.time(START_DISABLE_TIME1_HEATPUMP_HOUR,START_DISABLE_TIME1_HEATPUMP_MINUTE,0),[0,1,2,3,4]),    # bis 16.12.2011; 8:00 --> 15 Uhr bis 31.1.2017: --> 17:00 Uhr
    #         (datetime.time(END_DISABLE_TIME1_HEATPUMP_HOUR,END_DISABLE_TIME1_HEATPUMP_MINUTE,0),datetime.time(STOP_HEATING_MOTOR_TIME1,0,0),[0,1,2,3,4]),                           # 29.1.2022 -> keine Heizung, wenn Waermepumpen Strom ausgeschaltet ist !
    #         (datetime.time(9,00,0),datetime.time(STOP_HEATING_MOTOR_TIME2,00,0),[5,6]),                                                                                             # bis 16.12.2011; 9:30 --> 15 Uhr
    #         (datetime.time(21,00,0),datetime.time(23,59,59,999999)),
    #         (datetime.time(0,0,0),datetime.time(STOP_HEATING_MOTOR_TIME3,00,0))])  # test: (datetime.time(22,20,0),datetime.time(22,25,0)),# bis 16.12.2011; --> 3:30 Uhr bis 17.1.2017 --> 4:30 Uhr
    aEnableTimer = EnableTimer(ENABLE_HEATING_MOTOR,[(datetime.time(8,00,0),datetime.time(STOP_HEATING_MOTOR_TIME1,0,0),[0,1,2,3,4]),    # ab 18.1.2024 HeatPump connected to house power -> no shutdown times needed
            (datetime.time(9,00,0),datetime.time(STOP_HEATING_MOTOR_TIME2,00,0),[5,6]),
            (datetime.time(21,00,0),datetime.time(23,59,59,999999)),
            (datetime.time(0,0,0),datetime.time(STOP_HEATING_MOTOR_TIME3,00,0))])  # test: (datetime.time(22,20,0),datetime.time(22,25,0)),# bis 16.12.2011; --> 3:30 Uhr bis 17.1.2017 --> 4:30 Uhr
                                                                                                                    # bis 28.1.2020 --> 22:20 -> 4:30 Uhr
    # 7h + 5h == 12h Heizen
    aEnableTimerOnlyNight = EnableTimer(ENABLE_HEATING_MOTOR_NIGHT,[(datetime.time(21,00,0),datetime.time(23,59,59,999999)),
            (datetime.time(0,0,0),datetime.time(3,00,0))])  # test: (datetime.time(22,20,0),datetime.time(22,25,0)),# bis 16.12.2011; --> 3:30 Uhr
# TODO close mixer only if mixer was open before... --> save state: if open was used --> set flag, after mixer close --> clear flag
# TODO lasse heating control motor im sommer auch laufen, wenn aussentemperatur zu gross ist --> mixer ist zu !
# NEW TODO -> ausschalten des Mixer Close, falls gerade geheizt werden soll -> 
    aCloseMixerTimer = EnableTimer(ENABLE_MIXER_CLOSE,[(datetime.time(STOP_HEATING_MOTOR_TIME1,2,0),datetime.time(STOP_HEATING_MOTOR_TIME1,4,30),[0,1,2,3,4]),
                                                       (datetime.time(STOP_HEATING_MOTOR_TIME2,2,0),datetime.time(STOP_HEATING_MOTOR_TIME2,4,30),[5,6]),
                                                       (datetime.time(STOP_HEATING_MOTOR_TIME3,2,0),datetime.time(STOP_HEATING_MOTOR_TIME3,4,30))])
    aDateTimer = EnableTimer(ENABLE_DATE_HEATING_MOTOR,[(datetime.date(START_YEAR-1,11,1),datetime.date(START_YEAR,3,15)),
                                                        (datetime.date(START_YEAR,11,1),datetime.date(START_YEAR+1,3,15))]) #,
                                                        # (datetime.date(2010,11,1),datetime.date(2011,3,1))])    # year will be ignored !
#    aAntiFixingSwitchTimer = EnableTimer(ENABLE_ANTI_FIXING_SWITCH,[(datetime.datetime(2010,5,15,2,0,0),datetime.datetime(2010,5,15,2,5,0)),     # runs 5 minutes at 2am  
#                                                                    (datetime.datetime(2010,7,15,2,0,0),datetime.datetime(2010,7,15,2,5,0)),
#              # for testing:      (datetime.datetime(2010,1,5,19,51,0),datetime.datetime(2010,1,5,19,55,0)),                                                                                    
#                                                                    (datetime.datetime(2010,9,15,2,0,0),datetime.datetime(2010,9,15,2,5,0))])
    # 6.6.2021: run heating motor for anti fixing each day for 5 min !
    aAntiFixingSwitchTimer = EnableTimer(ENABLE_ANTI_FIXING_SWITCH,[(datetime.time(22,30,0),datetime.time(22,35,0))],returnNoneIfOff=True)     # runs 5 minutes at 2am   -> for testing: add current time interval: (datetime.time(15,50,30),datetime.time(15,52,0))
        # if anti fixing switch is not enabled -> return None -> this means disable OrNode for manual switch replacement in ENABLE_HEATING_MOTOR_ALL_INPUTS
    aMaxTempControlSwitch = ManualSwitch(MANUAL_SWITCH_MAX_TEMP_CONTROL)
    aHeatingIntervalSwitch = ManualSwitch(MANUAL_SWITCH_HEATING_ONLY_NIGHT)
    aHeatingIntervalDateSwitch = ManualSwitch(MANUAL_SWITCH_HEATING_DATE)  
    aHeatingIntervalValue = ValueSwitch(ENABLE_HEATING_MOTOR_VALUE,aEnableTimerOnlyNight,aEnableTimer,aHeatingIntervalSwitch)
    aHeatingIntervalDateNotSwitch = NotNode("NOT_"+aHeatingIntervalDateSwitch.get_name(), aHeatingIntervalDateSwitch)
    aHeatingIntervalDate = OrNode(ENABLE_HEATING_MOTOR_DATE, [aHeatingIntervalDateNotSwitch, aDateTimer])
# NEW TODO: DateAndValue -> use OR(AntiFixing,AND(aHeatingIntervalValue, aHeatingIntervalDate))    
    aHeatingIntervalDateAndValue = AndNode(ENABLE_HEATING_MOTOR_DATE_AND_VALUE,[aHeatingIntervalValue, aHeatingIntervalDate]) # aHeatingIntervalValue and (aDateTimer or aHeatingIntervalDateDisableSwitch)
    aHeatingManualOrAntiFixInterval = OrNode(ENABLE_HEATING_MOTOR_ALL_INPUTS,[aManualSwitchHeatingMotor, aAntiFixingSwitchTimer])  # DIESE ZEILE IST GEPATCHED !!! aManualSwitchHeatingMotor                            ####################################
    aHeatingMotor = SwitchRelais(SWITCH_MOTOR_HEATING,aRelaisMeasurement.switch_port_fcn(1))
    aOperatingHoursMotorHeating = OperatingHoursCounter(OPERATING_HOURS_MOTOR_HEATING,aHeatingMotor)
    aHeatingMotor.connect_input(aHeatingControl,aHeatingMotor.INPUT_ID)
    aHeatingMotor.connect_input(aHeatingManualOrAntiFixInterval,aHeatingMotor.INPUT_ID_MANUAL)
    aHeatingMotor.connect_input(aHeatingIntervalDateAndValue,aHeatingMotor.ENABLE_ID)   # until 17.1.2017: aEnableTimer
# TODO gulpx Zirkulationspumpe ausschalten wenn Waermepumpe wegen Vereisungsschutz ausgeschaltet wird --> aHeatingMotor.connect_input(aTempConverter/aHeatPummpControl,aHeatingMotor.AND_ENABLE_ID)     
# --> dazu muss die Sicherheits-Abschaltung der Waermepumpe in ein eigenes Objekt/Klasse verschoben werden und als Eingang in HeatPumpControl gesetzt werden 
# --> ggf. Moeglichkeit schaffen um Signale zu verunden und verordern um nur einen ENABLE_ID Eingang zu haben der aber mehrer Quellen zur Entscheidung ob 0 oder 1 hat
    #   Sollwert Mischer
    aDesiredValueMixerHeatingControl = DesiredValueForHeatingMixerControl(DESIRED_VALUE_MIXER_HEATING_CONTROL)
    aDesiredValueMixerHeatingControl.connect_input(aTempOutdoor,aDesiredValueMixerHeatingControl.INPUT_ID_OUTDOOR)
    aEnableMaxTempControl = AndNode(ENABLE_MIXER_MAX_TEMP_CONTROL, [aMaxTempControlSwitch, aHeatingMotor], 0)
    aDesiredValueMixerHeatingControl.connect_input(aEnableMaxTempControl,aDesiredValueMixerHeatingControl.INPUT_ID_ENABLE_MAX_TEMP_CONTROL)
    #   Heizung Mischer
    aTempMixer = TemperatureSource(MIXER_HEATING,aTempMeasurement.get_channel_fcn(3))
    aSwitchMixer = ThreeStateSwitchRelais(SWITCH_MIXER_HEATING,aRelaisMeasurement.three_switch_port_fcn(6,7))
    aMixerHeatingControl = HeatingMixerControl(MIXER_HEATING_CONTROL, lambda val : aSwitchMixer.set_closed(val), lambda : aSwitchMixer.is_closed(), lambda count : aSwitchMixer.update_open_tick_count(count))
    aManualSwitchMixer = ManualSwitchAutoReset(MANUAL_SWITCH_MIXER_HEATING)
    aMixerHeatingControl.connect_input(aTempMixer,aMixerHeatingControl.INPUT_ID_ACTUAL_VALUE)
    aMixerHeatingControl.connect_input(aDesiredValueMixerHeatingControl,aMixerHeatingControl.INPUT_ID_DESIRED_VALUE)
    aMixerHeatingControl.connect_input(aCloseMixerTimer,aMixerHeatingControl.INPUT_ID_CLOSE_SIGNAL)
    aMixerHeatingControl.connect_input(aHeatingMotor,aMixerHeatingControl.INPUT_ID_HEATING_MOTOR)       # or aHeatingIntervalValue ?
    aSwitchMixer.connect_input(aMixerHeatingControl,aSwitchMixer.INPUT_ID)
    aSwitchMixer.connect_input(aManualSwitchMixer,aSwitchMixer.INPUT_ID_MANUAL)
# TODO --> anscheinend ist heatingmotor nicht disabled wenn er durch timer disabled ist !    --> mixer laueft weiter auch wenn die cirkulationspumpe ausgeschaltet ist !
    aSwitchMixerEnable = OrNode(ENABLE_MIXER_MOVEMENT,[aHeatingMotor, aCloseMixerTimer])
    aSwitchMixer.connect_input(aSwitchMixerEnable,aSwitchMixer.ENABLE_ID)    # run mixer algorithm only if heating motor is on ! or aCloseMixerTimer is on
    
    # Waermepumpe Regelung
    aTempBuffer1 = TemperatureSource(BUFFER1,aTempMeasurement.get_channel_fcn(4))
    aTempBuffer2 = TemperatureSource(BUFFER2,aTempMeasurement.get_channel_fcn(5))
    aDesiredValueMixerHeatingControl.connect_input(aTempBuffer2,aDesiredValueMixerHeatingControl.INPUT_ID_CURRENT_VALUE)
    aTempHeatPumpOutgoingAir = TemperatureSource(HEATPUMP_OUTGOING_AIR,aTempMeasurement.get_channel_fcn(6))
    aTempConverter = TemperatureSource(CONVERTER,aTempMeasurement.get_channel_fcn(10)) #,7.5,True)  # TODO: patch if problems with CONVERTER temperature sensor occures: ,7.5,True)
    aTempRoom = TemperatureSource(ROOM,aTempMeasurement.get_channel_fcn(11))
    aHeatPummpControl = HeatPumpControl(HEATPUMP_CONTROL)
    aManualSwitchHeatPump = ManualSwitch(MANUAL_SWITCH_HEATPUMP)

    if g_support_heatpump_via_pv:
        # Enable Heatpump only if PV is available (via MANUAL_SWITCH) --> (MANUAL_SWITCH_HEATPUMP_ONLY_WITH_PV==1 && PV_TIME_INTERVAL) || MANUAL_SWITCH_HEATPUMP_ONLY_WITH_PV!=1
        aManualSwitchHeapumpOnlyWithPv = ManualSwitch(MANUAL_SWITCH_HEATPUMP_ONLY_WITH_PV)
        aLogicIsManualSwitchHeatpumpOnlyWithPv = CompareToValueNode(LOGIC_IS_MANUAL_SWITCH_HEATPUMP_ONLY_WITH_PV,aManualSwitchHeapumpOnlyWithPv,1,lambda a, b: a == b)
        aLogicIsNotManualSwitchHeatpumpOnlyWithPv = CompareToValueNode(LOGIC_IS_NOT_MANUAL_SWITCH_HEATPUMP_ONLY_WITH_PV,aManualSwitchHeapumpOnlyWithPv,1,lambda a, b: a != b)
        aPvTimeInterval = EnableTimer(ENABLE_HEAT_PUMP_FOR_PV,[(datetime.time(10,0,0),datetime.time(15,59,59,999999))])
        aEnableHeatpumpViaPV = AndNode(LOGIC_ENABLE_HEATPUMP_VIA_PV,[aLogicIsManualSwitchHeatpumpOnlyWithPv,aPvTimeInterval])
        aEnableHeatpumpTimer = OrNode(ENABLE_HEAT_PUMP,[aEnableHeatpumpViaPV,aLogicIsNotManualSwitchHeatpumpOnlyWithPv])
    else:
        # *** Heatingpump connected to Waermepumenstrom Zaehler -> Ausschaltzeiten zu Stosszeiten ***
        # time and weekday check needed --> Sa and So the voltage for the heating pump will allways be enabled
        # only Mo-Fr the heating pump voltage will be disabled from 11:30->13:00 and 17:30-->19:00
        # aEnableHeatpumpTimer = EnableTimer(ENABLE_HEAT_PUMP,[(datetime.time(0,0,0),datetime.time(START_DISABLE_TIME1_HEATPUMP_HOUR,28,59,999999)),
        #         (datetime.time(START_DISABLE_TIME1_HEATPUMP_HOUR,START_DISABLE_TIME1_HEATPUMP_MINUTE,0),datetime.time(END_DISABLE_TIME1_HEATPUMP_HOUR,1,59,999999),[5,6]),
        #         (datetime.time(END_DISABLE_TIME1_HEATPUMP_HOUR,END_DISABLE_TIME1_HEATPUMP_MINUTE,0),datetime.time(START_DISABLE_TIME2_HEATPUMP_HOUR,28,59,999999)),
        #         (datetime.time(START_DISABLE_TIME2_HEATPUMP_HOUR,START_DISABLE_TIME2_HEATPUMP_MINUTE,0),datetime.time(END_DISABLE_TIME2_HEATPUMP_HOUR,1,59,999999),[5,6]),
        #         (datetime.time(END_DISABLE_TIME2_HEATPUMP_HOUR,END_DISABLE_TIME2_HEATPUMP_MINUTE,0),datetime.time(23,59,59,999999))])
        aEnableHeatpumpTimer = EnableTimer(ENABLE_HEAT_PUMP,[(datetime.time(0,0,0),datetime.time(23,59,59,999999))])

    aSwitchHeatPump = SwitchRelais(SWITCH_HEATPUMP,aRelaisMeasurement.switch_port_fcn(2))
    aOperatingHoursHeatPump = OperatingHoursCounter(OPERATING_HOURS_HEATPUMP,aSwitchHeatPump,init_value=227.0*60.0*60.0)    # 227 hours since 16.1.2024 until 13.1.2024 15:30
    aHeatPummpControl.connect_input(aTempBuffer2,aHeatPummpControl.INPUT_ID_BUFFER)
    aHeatPummpControl.connect_input(aTempConverter,aHeatPummpControl.INPUT_ID_OUTGOING_AIR)
    aHeatPummpControl.connect_input(aManualSwitchHeatPump,aHeatPummpControl.INPUT_ID_MANUAL_SWITCH)
    aSwitchHeatPump.connect_input(aHeatPummpControl,aSwitchHeatPump.INPUT_ID)
    aSwitchHeatPump.connect_input(aManualSwitchHeatPump,aSwitchHeatPump.INPUT_ID_MANUAL)
    aSwitchHeatPump.connect_input(aEnableHeatpumpTimer,aSwitchHeatPump.ENABLE_ID)
    
    # Lueftungsanlage / Bypass
    aVentilationControl = VentilationControl(VENTILATION_CONTROL)
    aManualSwitchVentilation = ManualSwitch(MANUAL_SWITCH_VENTILATION)
    aSwitchVentilation = SwitchRelais(SWITCH_VENTILATION,aRelaisMeasurement.switch_port_fcn(3),aRelaisMeasurement.switch_port_for_not_value_fcn(4)) 
    aVentilationControl.connect_input(aTempConverter,aVentilationControl.INPUT_ID_OUTGOING_AIR)
    aVentilationControl.connect_input(aSwitchHeatPump,aVentilationControl.INPUT_ID_HEATPUMP_ON)     # TODO --> ggf. an SwitchHeatPump inkl manual switch haengen ?
    aSwitchVentilation.connect_input(aVentilationControl,aSwitchVentilation.INPUT_ID)               # TODO --> ggf. an SwitchHeatPump inkl manual switch haengen ?
    aSwitchVentilation.connect_input(aManualSwitchVentilation,aSwitchVentilation.INPUT_ID_MANUAL)
    
    # old: Ventil fuer Solarbetrieb der Waermepumpe  ==> neu seit 8.9.2019: Bypass Close Kanal, immer mit #3 für Bypass (siehe oben) verwenden
    aHeatPumpSolarValveControl = HeatPumpSolarValveControl(HEATPUMP_SOLAR_VALVE_CONTROL)
    aManualSwitchHeatPumpSolarValve = ManualSwitch(MANUAL_SWITCH_HEATPUMP_SOLAR_VALVE)
    aSwitchHeatPumpSolarValve = SwitchRelais(SWITCH_HEATPUMP_SOLAR_VALVE,aRelaisMeasurement.switch_virtual_port_fcn(4+100))
# TODO --> SwitchObject wird erst nach Control Object evaluiert !!! --> kein tickback value     
    #aHeatPumpSolarValveControl.connect_input(aSolarMotor,aHeatPumpSolarValveControl.INPUT_ID_SOLAR_FOR_BUFFER)
    aHeatPumpSolarValveControl.connect_input(aTempSolar,aHeatPumpSolarValveControl.INPUT_ID_COLLECTOR)
    aSwitchHeatPumpSolarValve.connect_input(aHeatPumpSolarValveControl,aSwitchHeatPumpSolarValve.INPUT_ID)
    aSwitchHeatPumpSolarValve.connect_input(aManualSwitchHeatPumpSolarValve,aSwitchHeatPumpSolarValve.INPUT_ID_MANUAL)

    # old: Booster or warm water circulation pump ==> neu seit 7.9.2013: Waermepumpe-Zirkulationspumpe wird damit angesteuert, d. h. einschalten wenn Waermepumpe laueft !
# TODO: als virtuellen Switch realisieren, d. h. ist nicht mit Hardware verbunden --> RelaisMeasurement sollte virtual port haben !!!
    aSwitchBooster = SwitchRelais(SWITCH_BOOSTER,aRelaisMeasurement.switch_port_fcn(5))
    aManualSwitchBooster = ManualSwitch(MANUAL_SWITCH_BOOSTER)
    aSwitchBooster.connect_input(aSwitchHeatPump,aSwitchBooster.INPUT_ID)
    aSwitchBooster.connect_input(aManualSwitchBooster,aSwitchBooster.INPUT_ID_MANUAL)
    
    # other (unused) sensors
    aWarmWater = TemperatureSource(WARM_WATER,aTempMeasurement.get_channel_fcn(7))
    aHeatCreator = TemperatureSource(HEAT_CREATOR,aTempMeasurement.get_channel_fcn(8))
    aIngoingAir = TemperatureSource(INGOING_AIR,aTempMeasurement.get_channel_fcn(9))
    if bNewHeatingcontrolBoardDetected:
        aTestSensor = TemperatureSource(TESTSENSOR_PT1000,aTempMeasurement.get_channel_fcn(15))
    
    # debugging
    aCounter = TickCounterSource(TICK_COUNTER)
    aArduinoCounter = TemperatureSource(ARDURINO_COUNT,aTempMeasurement.get_channel_fcn(12))
    
    # watchdog
    aWatchdog = Watchdog(WATCHDOG)
    
    # register object for control engine 
    aControlEngine = ControlEngine()
    # WARNING: order is very important !
    aControlEngine.append(aTempMeasurement)
    aControlEngine.append(aLightSensor1)
    aControlEngine.append(aLightSensor2)
    aControlEngine.append(aTempSolar)
    aControlEngine.append(aTempSolarBuffer)
    aControlEngine.append(aTempOutdoor)
    aControlEngine.append(aTempMixer)
    aControlEngine.append(aTempBuffer1)
    aControlEngine.append(aTempBuffer2)
    aControlEngine.append(aTempHeatPumpOutgoingAir)
    aControlEngine.append(aWarmWater)
    aControlEngine.append(aHeatCreator)
# TODO: Reihenfolge bestimmt Spaltenfolge im Daten-File !!! --> achtung spalten nummern werden inkompatibel   
    aControlEngine.append(aTempConverter)           # since 7.11.2010: old position: aIngoingAir
    aControlEngine.append(aTempRoom)

    aControlEngine.append(aSolarControl)
    aControlEngine.append(aHeatingControl)
    aControlEngine.append(aDesiredValueMixerHeatingControl)
    aControlEngine.append(aMixerHeatingControl)
    aControlEngine.append(aHeatPummpControl)
    aControlEngine.append(aVentilationControl)
    aControlEngine.append(aHeatPumpSolarValveControl)
    
    aControlEngine.append(aEnableTimer)
    aControlEngine.append(aEnableTimerOnlyNight)
    aControlEngine.append(aEnableHeatpumpTimer)     # new since 9.11.2010    
    aControlEngine.append(aCloseMixerTimer)         # new sinde 3.1.2019
    aControlEngine.append(aDateTimer)               # new sinde 4.1.2019
    aControlEngine.append(aAntiFixingSwitchTimer)   # new sinde 4.1.2019
    
    aControlEngine.append(aManualSwitchSolarMotor)
    aControlEngine.append(aManualSwitchHeatingMotor)
    aControlEngine.append(aHeatingIntervalSwitch)
    aControlEngine.append(aHeatingIntervalDateSwitch)
    aControlEngine.append(aHeatingIntervalDateNotSwitch)  # new since 4.1.2019  
    aControlEngine.append(aHeatingIntervalValue)
    aControlEngine.append(aHeatingIntervalDate)           # new since 4.1.2019
    aControlEngine.append(aHeatingIntervalDateAndValue)   # new since 4.1.2019
    aControlEngine.append(aHeatingManualOrAntiFixInterval)# new since 4.1.2019
    aControlEngine.append(aMaxTempControlSwitch)          # new since 6.1.2019
    aControlEngine.append(aEnableMaxTempControl)          # new since 6.1.2019
    aControlEngine.append(aManualSwitchMixer)
    aControlEngine.append(aManualSwitchHeatPump)
    aControlEngine.append(aManualSwitchVentilation)
    aControlEngine.append(aManualSwitchHeatPumpSolarValve)
    aControlEngine.append(aManualSwitchBooster)

    aControlEngine.append(aSolarMotor)
    aControlEngine.append(aHeatingMotor)
    aControlEngine.append(aSwitchMixer)
    aControlEngine.append(aSwitchMixerEnable)           # new since 5.1.2019
    aControlEngine.append(aSwitchHeatPump)
    aControlEngine.append(aSwitchVentilation)               # TODO: GULP --> nur wenn waermepumpe wirklich laueft einschalten 
    aControlEngine.append(aSwitchHeatPumpSolarValve)
    aControlEngine.append(aSwitchBooster)                   # TODO: GULP --> einschalte wenn Waermepumpe laeuft ! 
# TODO --> Beobachter Objekt implementieren um festzustelen dass Waermepumpe laeuft --> HEAT_CREATOR > 25 Grad ==> Waermepumpe laeuft, HEAT_CREATOR < 20 fuer laengere Zeit ==> Waermepumpe aus    
    
    aControlEngine.append(aIngoingAir)          # new since 7.11.2010: moved to this position for new CONVERTER value 
    
    aControlEngine.append(aTimeline)
    
    aControlEngine.append(aRelaisMeasurement)   # update state of relais manager after tick    
    
    aControlEngine.append(aCounter)
    aControlEngine.append(aArduinoCounter)
    if g_bUseWatchdog:
        aControlEngine.append(aWatchdog)
    if bNewHeatingcontrolBoardDetected:
        aControlEngine.append(aTestSensor)

    aControlEngine.append(aOperatingHoursHeatPump)
    aControlEngine.append(aOperatingHoursMotorHeating)
    aControlEngine.append(aOperatingHoursMotorSolar)

    if g_support_heatpump_via_pv:
        aControlEngine.append(aManualSwitchHeapumpOnlyWithPv)                # new since 5.10.2024
        aControlEngine.append(aLogicIsManualSwitchHeatpumpOnlyWithPv)        # new since 5.10.2024
        aControlEngine.append(aLogicIsNotManualSwitchHeatpumpOnlyWithPv)     # new since 5.10.2024
        aControlEngine.append(aPvTimeInterval)                               # new since 5.10.2024
        aControlEngine.append(aEnableHeatpumpViaPV)                          # new since 5.10.2024
    
    aControlEngine.append(aEsp32HttpTemperature)

    # *** IMPORTANT: ***
    # add new control items at the end of this list,
    # otherwise the data format of the csv file will be not compatible
    # new columns at the end of the row are backward compatible !

    return aControlEngine

LINE = "======================================================================"
nl = "\n"

g_http_port_no = 80

aControlEngine = None       # global variable for CTRL-C handler

def run_control(thread_communication_context,aLock):
    """
        The heating control loop.
    """
    bContinue = True
    sActFileName,aActDate,bNewFile = map_date_to_filename(g_sLogFile,is_debug())
    append_to_file(sActFileName,[],sPrefix="#restart heizung.py "+str(datetime.datetime.today()))
    iCount = 0
    aControlEngine = None
    while iCount<10 and bContinue:
        #if True:
        try:
            #print( "running control... count="+str(iCount) )
            append_to_logfile("running control... count="+str(iCount),is_debug())
            #if not os.path.exists("data"): #os.access("data",os.W_OK):
            #    os.mkdir("data")
            aControlEngine = configure_control()
            if g_bDumpDependencyGraph:
                # enable this block for creating documentation
                print("=============================================")
                print("GRAPH:", aControlEngine.get_graph())
                print("=============================================")
                print("DOT:", aControlEngine.get_dot_data())
            global g_aControlEngine
            g_aControlEngine = aControlEngine
            bContinue = not aControlEngine.run(g_sLogFile,thread_communication_context,aLock)
            #print "STOP control engine !",bContinue,iCount
        except Exception as aExcp:
            iCount += 1
            print( "EXCEPTION CONTROL",aExcp,iCount )
            output = StringIO()
            traceback.print_exc(file=output)
            sActFileName,aActDate,bNewFile = map_date_to_filename(g_sLogFile,is_debug())
            append_to_file(sActFileName+".log",[LINE+nl+str(time.strftime("%d.%m.%Y;%H:%M:%S"))+nl+"CONTROL_EXCEPTION:"+str(aExcp)+" "+str(output.getvalue())],sPrefix="#")
            print( output.getvalue() )
            if aControlEngine is not None:
                aControlEngine.run_persistence()
                aControlEngine.finish()
                aControlEngine = None
            time.sleep(5)       # give the hardware time to reset ?
            #gc.collect()
    #print "try stop control server",bContinue
    if bContinue:
        # in the case of a connection timeout inform http server to stop and fully restart 
        #if qWrite!=None:
            #print "--> WRITE STOP to HTTP server"
            #qWrite.put("STOP")
            try:
                import urllib2
            except ModuleNotFoundError as exc:
                import urllib.request as urllib2
            urllib2.urlopen("http://127.0.0.1:"+str(g_http_port_no)+"/;stop")
    #print "STOP control server"
    append_to_logfile("STOP CONTROL_THREAD",is_debug())
    if aControlEngine is not None:
        aControlEngine.run_persistence()
        aControlEngine.finish()
    aControlEngine = None

def run_server(bTest,bUseHttpDefaultPort,qWrite=None,qRead=None):
    """
        The http server loop.
    """
    import simplesrv
    port = 8008 if bTest else 8001    # 5.4.2023 changed from 8000 -> 8001 because of Grafana support
    if bUseHttpDefaultPort:
        port = 80
    global g_http_port_no
    g_http_port_no = port
    bContinue = True
    while bContinue:
        try:
            #print "running server..."
# TODO --> maybe: server alle n Minuten/Stunden neu starten... ? --> falls Probleme mit haengender HTTP Kommunikation weiter auftritt...
            bContinue = not simplesrv.run_server(port,qWrite,qRead,bTest,bUseHttpDefaultPort)      # if server returns we want to stop the threading   
                                                                                              # restart server only if exception occured !
        except Exception as aExcp:
            print( "EXCEPTION SERVER",aExcp )
            sActFileName,aActDate,bNewFile = map_date_to_filename(g_sLogFile,is_debug())
            append_to_file(sActFileName,["SERVER_EXCEPTION:"+str(aExcp)],sPrefix="#")
    #print "STOP httpd server"
    append_to_logfile("STOP HTTP_SERVER_THREAD",is_debug())
    
def run_rs232_test():
    """
        Function for testing of the communication with the UART
    """
    s1 = serial.Serial("/dev/ttySAC1")
    s2 = serial.Serial("/dev/ttySAC2")
    iCount = 0
    while True:
        s1.write("from /dev/ttySAC1 count="+str(iCount)+"\n")
        s2.write("from /dev/ttySAC2 count="+str(iCount)+"\n")
        iCount += 1
        time.sleep(0.5)

# see: https://realpython.com/python-sockets/ 
# and: https://github.com/realpython/materials/tree/master/python-sockets-tutorial
def run_socket_server(debugging,qWrite,qRead):
    """
        The socket server loop
    """
    host = "localhost"
    port = 42424

    sel = selectors.DefaultSelector()

    def accept_wrapper(sock):
        conn, addr = sock.accept()  # Should be ready to read
        if debugging:
            print(f"Accepted connection from {addr}")
        conn.setblocking(False)
        message = libserver.Message(sel, conn, addr, debugging=debugging, qWrite=qWrite, qRead=qRead)
        sel.register(conn, selectors.EVENT_READ, data=message)

    lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Avoid bind() exception: OSError: [Errno 48] Address already in use
    #lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    lsock.bind((host, port))
    lsock.listen()
    print(f"Listening on {(host, port)}")
    lsock.setblocking(False)
    sel.register(lsock, selectors.EVENT_READ, data=None)

    try:
        while True:
            events = sel.select(timeout=None)
            for key, mask in events:
                if key.data is None:
                    accept_wrapper(key.fileobj)
                else:
                    message = key.data
                    try:
                        message.process_events(mask)
                    except Exception:
                        print(
                            f"Main: Error: Exception for {message.addr}:\n"
                            f"{traceback.format_exc()}"
                        )
                        message.close()
    except KeyboardInterrupt:
        print("Caught keyboard interrupt, exiting")
    finally:
        sel.close()

def run_communication(thread_communication_context,aLock,qWrite,qRead,qSocketWrite,qSocketRead):
    """
        The pipe communication loop
    """
    # wait for other thread to update the thread_communication_context
    time.sleep(3)
    bStop = False
    while not bStop:
        with aLock:
            if thread_communication_context is not None:
                if 'fcn_poll_for_commands' in thread_communication_context and thread_communication_context['fcn_poll_for_commands'] is not None:
                    bStop = thread_communication_context['fcn_poll_for_commands'](qWrite,qRead,qSocketWrite,qSocketRead)
                if 'fcn_set_stop' in thread_communication_context and thread_communication_context['fcn_set_stop'] is not None:
                    thread_communication_context['fcn_set_stop'](bStop)
        time.sleep(0.1)

def run(bRunWithThreads=False):
    bWithThreeProcs = False
    #print "RUN MAIN now..."
    aQueueServerToControl = Queue()
    aQueueControlToServer = Queue()
    aQueueSocketToControl = Queue()
    aQueueControlToSocket = Queue()
    if bRunWithThreads:
        serverProcId = thread.start_new_thread( run_server, (g_bTest or g_bSimPort,g_bUseHttpDefaultPort,aQueueServerToControl,aQueueControlToServer) )
    else:
        # currently obsolet and not consistently implemented for other threads !
        aServerProc = Process(target=run_server,args=(g_bTest or g_bSimPort,g_bUseHttpDefaultPort,aQueueServerToControl,aQueueControlToServer))
        aServerProc.start()
    aLock = threading.Lock()
    thread_communication_context = {'fcn_set_stop': None, 'fcn_poll_for_commands': None}
    serverSocketId = thread.start_new_thread( run_socket_server, (False,aQueueSocketToControl,aQueueControlToSocket) )
    serverCommunicationId = thread.start_new_thread( run_communication, (thread_communication_context,aLock,aQueueControlToServer,aQueueServerToControl,aQueueControlToSocket,aQueueSocketToControl) )
    # test rs232 communication via UART
    if False:
        aRs232TestProc = Process(target=run_rs232_test)
        aRs232TestProc.start()
    if bWithThreeProcs:
        aControlProc = Process(target=run_control,args=(aQueueControlToServer,aQueueServerToControl))
        aControlProc.start()    
        while aControlProc.is_alive() or aServerProc.is_alive():
            #print "ping",aServerProc.is_alive(),aControlProc.is_alive()
            sys.stdout.flush() 
            time.sleep(0.1)
        #print "STOP main process"
        append_to_logfile("STOP MAIN PROCESS",is_debug())
    else:
        run_control(thread_communication_context,aLock)
    
def ShowUsage():
    print() 
    print( "heating control programm, available options:" )
    print( "  -s : silent modus, no debugging output" )
    print( "  -i : use simulation port 8008 for http" )
    print( "  -p : use default port 80 for http" )
    print( "  -t : test without hardware (simulate hardware)" )
    print( "  -e : use exclusive access modus for RS232 interface" )
    print( "  -c : run with history cache" )
    print( "  -d : run without history cache" )
    print( "  -w : disable watchdog in python program" )
    print( "  -g : dump dependency info at startup (for documentation)" )
    print( "  -h : show this help" )

def signal_handler(sig, frame):
    try:
        print('Ctrl+C pressed !') #,sig, frame)
        g_aControlEngine.run_persistence()
    except Exception as exc:
        print("EXCEPTION in signal handler:", exc)
    sys.exit(0)

# *************************************************************************

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    
    bRun = True
    if '-s' in sys.argv:   # silent modus --> no debugging output !
        set_debug(False)
    if '-i' in sys.argv:   # test with simulation port no 8008
        set_sim_port(True)
    if '-p' in sys.argv:   # use the default port no 80 for http
        set_default_http_port(True)
    if '-t' in sys.argv:   # test without hardware (simulate hardware)
        set_test(True)
    if '-e' in sys.argv:   # use exclusive modus for RS232 interface 
        # -> this heizungsregelungs process locks the RS232 interface 
        #    to communicate with the heating control board for the
        #    whole time the process is running.
        #    Otherwise the RS232 interface will be requested only
        #    if needed for communication and afterwards the interface
        #    will be released again, so that another process can use 
        #    it to communicate with the heating control board
        set_exclusive_rs232(True)
    if '-c' in sys.argv:   # run with history cache
        set_use_cache(True)
    if '-d' in sys.argv:   # run without history cache
        set_use_cache(False)
    if '-w' in sys.argv:   # disable watchdog in python program
        set_use_watchdog(False)
    if '-g' in sys.argv:
        set_dump_dependency_graph(True)
    if '-h' in sys.argv:
        ShowUsage()
        bRun = False
    if bRun:
        run(g_bUseThreads)
