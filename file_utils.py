"""
    Some helper functions to handle files.
"""

import sys
import os
import datetime
import pickle

SEP = ";"

PERSISTENCE_EXTENSION = ".persistence"

g_sLogFile = "data/temperatures.csv"
g_sPvLogFile = "data/pv_facility.csv"
g_sDailyPvValuesFile = "data/pv_facility_daily.txt"

g_sPersistencePath = "persistence"
g_sCachePath = "cache"

def list_to_csv_line(aList):
    s = ""
    for e in aList:
        if len(s)>0:
            s += SEP
        s += str(e) if e!=None else ""
    return s

def append_to_file(sFileName,aData,sPrefix="",sPostfix="",sHeaderForNewFile=None):
    if os.path.exists(sFileName):
        aFile = open(sFileName,"a")
    else:
        #print "file",sFileName,"not found --> created!"
        verify_path(sFileName)
        aFile = open(sFileName,"w")
    txt = list_to_csv_line(aData) if isinstance(aData, (list, tuple)) else str(aData)
    aFile.write(sPrefix+txt+sPostfix+"\n")
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

def process_for_pickle(s):
    if  sys.version_info.major==3:
        #bs = bytes(s,"latin-1")  # for python 3
        bs = bytes(s)  # for python 3
    else:
        bs = s # for python 2
    return bs

def write_data(sFileName, data):
    s = pickle.dumps(data)
    if  sys.version_info.major==3:
        write_bytes_file(sFileName,s)
    else:
        write_text_file(sFileName,s)
    
def read_data(sFileName):
    if  sys.version_info.major==3:
        s = read_bytes_file(sFileName)
    else:
        s = read_text_file(sFileName)
    if len(s)>0:
        data = pickle.loads(process_for_pickle(s))
        return (True,data)
    return (False,None)

def ensure_path_for_file(sFileName):
    path,name = os.path.split(sFileName)
    if not os.path.exists(path):
        os.makedirs(path)

def write_text_file(sFileName,data):
    try:
        ensure_path_for_file(sFileName)
        f = open(sFileName,"w")
        s = f.write(data)
        f.close()
    except Exception as exc:
        print( "EXCEPTION in write_text_file()",exc )

def read_text_file(sFileName):
    try:
        f = open(sFileName,"r")
        s = f.read()
        f.close()
        return s
    except Exception as exc:
        print( "EXCEPTION in read_text_file()",exc )
        return ""

def write_bytes_file(sFileName,data):
    try:
        ensure_path_for_file(sFileName)
        f = open(sFileName,"bw")
        s = f.write(data)
        f.close()
    except Exception as exc:
        print( "EXCEPTION in write_bytes_file()",exc )

def read_bytes_file(sFileName):
    try:
        f = open(sFileName,"br")
        s = f.read()
        f.close()
        return s
    except Exception as exc:
        print( "EXCEPTION in read_bytes_file()",exc )
        return ""

def check_path(sPath):
    if not os.path.exists(sPath):
        os.makedirs(sPath)

