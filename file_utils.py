"""
    Some helper functions to handle files.
"""

import os
import datetime

SEP = ";"

g_sLogFile = "data/temperatures.csv"
g_sPvLogFile = "data/pv_facility.csv"

def list_to_csv_line(aList):
    s = ""
    for e in aList:
        if len(s)>0:
            s += SEP
        s += str(e) if e!=None else ""
    return s

def append_to_file(sFileName,aData,sPrefix="",sHeaderForNewFile=None):
    if os.path.exists(sFileName):
        aFile = open(sFileName,"a")
    else:
        #print "file",sFileName,"not found --> created!"
        verify_path(sFileName)
        aFile = open(sFileName,"w")
    aFile.write(sPrefix+list_to_csv_line(aData)+"\n") 
    aFile.close()

def verify_path(sFileNameWithPath):
    sNormPath = os.path.normpath(sFileNameWithPath)
    sHead,sTail = os.path.split(sNormPath)
    sDirs = sHead.split(os.sep)
    sCheckPath = ""
    for e in sDirs:
        if e=="":               # if first item is empty --> handle as absolute path !
            sCheckPath = "/"
        sCheckPath += e
        if not os.path.exists(sCheckPath): #os.access(sCheckPath,os.W_OK):
            os.mkdir(sCheckPath)
        sCheckPath += os.sep
    return sNormPath
    
def two_digits(iNumber):
    return "%02d" % (iNumber,)
    
def add_sdcard_path_if_available(sFileNameWithPath):
    sRet = sFileNameWithPath
    # IMPORTANT: to use the USB-Memory-Stick
    # -> name the USB Stick to "USB_DATA" and create directory "heating_control" !
#   heizung control is running as root !!!
#   data_dir = "/media/"+getpass.getuser()+"/USB_DATA/heating_control"
    data_dir = "/media/pi/USB_DATA/heating_control"
    if os.path.exists(data_dir):
        sRet = data_dir+"/"+sRet
    elif os.path.exists("/sdcard"):
        sRet = "/sdcard/"+sRet
    return sRet

def map_date_to_filename(sFileName,is_debug,aOldDate=None):
    bNewFile = False
    now = datetime.datetime.today()
    sHead,sTail = os.path.split(sFileName)
    sName,sExt = os.path.splitext(sTail)
    if is_debug:
        # add test prefix to not interfere with real data file 
        sName = 'test_'+sName
    sNewFileName = sHead+os.sep+str(now.year)+os.sep+two_digits(now.month)+os.sep+sName+"_"+("%04d_%02d_%02d" % (now.year,now.month,now.day))+sExt
    if aOldDate!=None and aOldDate.day!=now.day:   # is new date ? --> day changes ?
        sNewFileName = verify_path(sNewFileName)
        bNewFile = True
    sNewFileName = add_sdcard_path_if_available(sNewFileName)
    return (sNewFileName,now,bNewFile)

def get_last_count_from_file(sFileName):
    ret = None
    aFile = open(sFileName,"r")
    sData = aFile.readlines()
    iIndex = len(sData)-1
    while iIndex>=0 and ret==None:
        sLine = sData[iIndex]
        if not sLine.startswith("#") and len(sLine)>0:
            aItems = sLine.split(SEP)
            if len(aItems)>0:
                ret = int(aItems[0])+1
        iIndex -= 1    
    aFile.close()
    return ret if ret!=None else 0

def append_to_logfile(sText,is_debug):
    sActFileName,aActDate,bNewFile = map_date_to_filename(g_sLogFile,is_debug)
    append_to_file(sActFileName+".log",[sText],sPrefix="#")
    #print(sText)
