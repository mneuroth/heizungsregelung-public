"""
    Classes to average and cache a plot history.
    
    Value-Producer --> Averager     --> Plot-Cache
    
       value rate      reduced rate
"""

import sys
import os
import os.path
import pickle

CACHE_EXTENSION = ".cache"

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

# *************************************************************************    
class LimitedList(list):
    """
        Class to limit the number of 
        elements in a list.
    """
    
    def __init__(self,iMaxSize):
        self.iMaxSize = iMaxSize
        
    def get_data(self):
        return self[:]
        
    def set_data(self,data):
        self[:] = data
        
    def append(self,value):
        list.append(self,value)
        # limit the number of elements in the array
        # to the given number of elements
        if len(self)>self.iMaxSize:
            # resize array
            self[:] = self[len(self)-self.iMaxSize:]
    
# *************************************************************************    
class PipeNode:
    """
        Class to represent a pipe node.
        Each node receives a value via the receive() method
        and should forward the (processed) value via the
        forward() method.
        
        A real pipe node must be derived from this class
        and overwrite the receive() method.
        
        
        PipeNodeA                               PipeNodeB        
           A.forward(valueA) --> B.receive(valueA) ... B.forward(valueB) -->
    """
    
    def __init__(self,aNextNode=None):
        """
            Constructor with next pipe node 
            to forward values
        """
        self.aNextNode = aNextNode
    
    def receive(self,value):
        """
            Receive a value in the pipe.
            Must be overwritten to process.
        """
        # default behaviour: forward value
        self.forward(value)
            
    def forward(self,value):
        """
            Forward value to next pipe node
        """
        if self.aNextNode!=None:
            self.aNextNode.receive(value)            
        
# *************************************************************************    
class PlotHistoryCache(LimitedList,PipeNode):
    
    def __init__(self,sCacheName,sPersitencePath,iMaxSize,aNextNode=None):
        LimitedList.__init__(self,iMaxSize)
        PipeNode.__init__(self,aNextNode)
        self.sCacheName = sCacheName
        self.sPersitencePath = sPersitencePath
        self._load_data()
        check_path(self.sPersitencePath)
        
    def run_persistence(self):
        self._save_data()

    def receive(self,value):
        self.append(value)
        # forward value to next node
        self.forward(value)
        self._save_data()
        
    def _load_data(self):
        sFileName = self._get_filename()
        if os.path.exists(sFileName):
            if  sys.version_info.major==3:
                s = read_bytes_file(sFileName)
            else:
                s = read_text_file(sFileName)
            if len(s)>0:
                if  sys.version_info.major==3:
                    #bs = bytes(s,"latin-1")
                    bs = bytes(s)
                else:
                    bs = s
                data = pickle.loads(bs)
                self.set_data(data)
        
    def _save_data(self):
        s = pickle.dumps(self.get_data())
        sFileName = self._get_filename()
        if  sys.version_info.major==3:
            write_bytes_file(sFileName,s)
        else:
            write_text_file(sFileName,s)

    def _get_filename(self):
        return self.sPersitencePath+os.sep+self.sCacheName+CACHE_EXTENSION
        
# *************************************************************************    
class Averager(PipeNode):
    """
        Class to average a given number
        of elements
    """

    def __init__(self,iNumberToAverage,aNextNode=None):
        PipeNode.__init__(self,aNextNode)
        self.iNumberToAverage = iNumberToAverage
        self.lstValues = [ 0 for e in range(self.iNumberToAverage) ]
        self.iCurrentPos = 0
        
    def receive(self,value):
        self.lstValues[self.iCurrentPos] = value
        self.iCurrentPos += 1
        # do we need to average ?
        if self.iCurrentPos>=self.iNumberToAverage:
            self.iCurrentPos = 0
            if type(value)==int or type(value)==float:
                dSum = sum(self.lstValues)
                dResult = dSum/float(self.iNumberToAverage)
                self.forward(dResult)
            else:
                # if type of value is not a number, just forward last value
                self.forward(value)
            
# *************************************************************************    
def NoneToInt(value):
    if value==None:
        return 0
    elif type(value)==str:
        return int(value)
    else:
        return value

def CreateAveragedHistoryCache(sCacheName,cacheSize=240,averageCount=72,sPersitencePath="."+os.sep+"cache"):
    # 10 samples per hour --> 240 samples per day = 24h
    # 1 sample per 6 min --> average over 6*60/5 = 72
#TODO --> fix path for new raspberry pi !    
    if os.path.exists("/sdcard"):
        sPersitencePath = "/sdcard/cache"
    cache = PlotHistoryCache(sCacheName,sPersitencePath,cacheSize)    
    averager = Averager(averageCount,cache)
    producer = PipeNode(averager)   
    return cache, lambda newValue : producer.receive(NoneToInt(newValue))

# *************************************************************************    
def test():
    cache = PlotHistoryCache(100)    
    averager = Averager(10,cache)
    producer = PipeNode(averager)
    for i in range(1100):
        producer.receive(i)
    print( cache,len(cache) )
        
# *************************************************************************    
if __name__ == '__main__':
    test()
