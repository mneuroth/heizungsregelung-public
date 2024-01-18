# -*- coding: utf-8 -*-
"""
    Implement a request handler for heating control application
    using the SimpleHTTPRequestHandler as base class.
    
    http://127.0.0.1:80/  
    http://heizungsregelung:80/
    
    TODO: ggf. use as Bluetooth Server 
    ggf. web proxy verwenden ? --> http://stackoverflow.com/questions/4488338/webview-android-proxy
    
    Android                                                                    |  Raspery Pi
    
    WebView --> http://127.0.0.1:8001/ --> Lokaler Server  --> bluetooth  -->  | HTTP/Bluetooth Server  --> heizungs_control
    
    Bluetooth-Server:
    while True:
       socket.listen()
       client = socket.accept()
       communicate_with_client(client)
    
    def communicate_with_client(client):
        while not client.close():
            req = read_request(client)
            data = get_data_from_control(req)
            write_response(client,data)
    
    Android WebServer class: 
        http://www.integratingstuff.com/2011/10/24/adding-a-webserver-to-an-android-app/
        http://stackoverflow.com/questions/11763475/how-to-start-an-android-web-server-service                
        
        http://stefanfrings.de/qtwebapp/
        
    plot:    
        http://woork.blogspot.de/2009/03/useful-scripts-to-plot-charts-in-web.html
        https://www.chartjs.org/docs/latest/getting-started/usage.html
        
    TODOs:
      * Bluetooth proxy
      * Plot-Data Cache (server oder heizung ?)

function JSAction(val) {
  if (val==1) {
    alert("1 selected");
  } else {
    alert("2 selected");
  }
}

function JSStop() {
  var xhReq = new XMLHttpRequest();
//  xhReq.open("GET", "http://127.0.0.1:8000/stop", false);
  xhReq.open("GET", "stop", false);
  xhReq.send();
}

function drawCanvas() {
    var canvas = document.getElementById('myCanvas');
    if(canvas.getContext) {
        var context = canvas.getContext('2d');
        context.fillStyle = 'rgb(255,0,0)';
        context.fillRect(20,40,800,260);
    }
    //  alert("xxx");

}

	var lineChartData = {
		labels : [1,2,3,4,5,6,7],
		datasets : [
			{
				fillColor : "rgba(255,255,255,1)",
				strokeColor : "rgba(0,0,0,1)",
				pointColor : "rgba(0,0,0,1)",
				//pointStrokeColor : "#fff",
				data : [65,59,90,81,56,55,40]
			},
		]		
	}
    var lineOptions = {
        animation: false,
        bezierCurve: false,
        scaleShowGridLines: false,
        scaleLineColor : "rgba(0,0,0,1)",
    };
////Get the context of the canvas element we want to select
//var ctx = document.getElementById("myCanvas").getContext("2d");
//var myNewChart = new Chart(ctx).Line(lineChartData,lineOptions);

#MANUAL_SWITCH_MIXER_HEATING
#MANUAL_SWITCH_BOOSTER
#MANUAL_SWITCH_MOTOR_HEATING
#MANUAL_SWITCH_VENTILATION
#MANUAL_SWITCH_MOTOR_SOLAR
#MANUAL_SWITCH_HEATPUMP
#MANUAL_SWITCH_HEATPUMP_SOLAR_VALVE

#ENABLE_HEAT_PUMP
#ENABLE_HEATING_MOTOR

#SWITCH_MIXER_HEATING
#SWITCH_BOOSTER
#SWITCH_MOTOR_HEATING
#SWITCH_VENTILATION
#SWITCH_MOTOR_SOLAR
#SWITCH_HEATPUMP
#SWITCH_HEATPUMP_SOLAR_VALVE

#MIXER_HEATING_CONTROL
#HEATING_CONTROL
#VENTILATION_CONTROL
#SOLAR_CONTROL
#HEATPUMP_CONTROL
#HEARTPUMP_SOLAR_VALVE_CONTROL

# speziell
#MASTER_ID
#TICK_COUNTER
#ARDUIONO_COUNT
#WATCHDOG
#ACT_TICK

"""

import time
#import datetime
import sys
import os
import os.path

try:
    from SimpleHTTPServer import SimpleHTTPRequestHandler as SimpleHTTPRequestHandler
except:
    from http.server import SimpleHTTPRequestHandler as SimpleHTTPRequestHandler
try:    
    from BaseHTTPServer import HTTPServer
except:
    from http.server import HTTPServer
try:    
    import urlparse
except:
    import urllib.parse as urlparse
try:
    from cStringIO import StringIO
except ImportError:
    try:
        from StringIO import StringIO
    except:
        from io import StringIO, BytesIO

DEFAULT_PORT = 0

try:
    if os.name=="posix" and sys.platform=="darwin":
        # ligthblue: http://lightblue.sourceforge.net/#downloads
        # patch for ligthblue: https://trac.macports.org/ticket/38666
        import lightblue as bluetooth
        from lightblue import socket as BluetoothSocket
        import threading
        DEFAULT_PORT = 0
    else:
        import bluetooth
        from bluetooth import BluetoothSocket
        import threading
        DEFAULT_PORT = 0
    g_bFoundBluetooth = True
except ImportError:
    g_bFoundBluetooth = False
except NotImplementedError:
    g_bFoundBluetooth = False

# *************************************************************************

__version__ = "1.7"
__date__    = "18.1.2024"

# *************************************************************************

# global variables
g_http_port_no = 80

def _hex(val):
    return  "%0.2X" % (val,)
    
def str_to_value(val):
    if type(val)==int:
        return val
    elif type(val)==float:
        return int(val)
    elif val==None:
        return None
    elif val=="1":
        return 1
    elif val=="-1":
        return -1
    else:
        return 0

def create_dataset(data,rgbColor,aTimelineForXAxis=None):
    sData = ""
    sLabels = ""
    i = 1
    for e in data:
        if len(sData)>0:
            sData += ","
            #if i % 5 == 0:
            sLabels += ","
        sData += "%3.2f" % (e,) # str(e)
        #if i % 5 == 0:
        IGNORE_LABEL_COUNT = 24
        sLabel = str(i) if i % IGNORE_LABEL_COUNT == 0 else ""
        if aTimelineForXAxis!=None and (i % IGNORE_LABEL_COUNT == 0):
            if len(aTimelineForXAxis)>i-1:
                sLabel = str(aTimelineForXAxis[i-1])[11:16]  # remove something from 2019-01-06 17:42.32.1234
            else:
                sLabel = str(i)
        else:
            sLabel = ""
        sLabels += '"' + sLabel + '"'
        i += 1
    sData = "["+sData+"]"
    sLabels = "["+sLabels+"]"
    sRet = """
        	{
                fillColor : "rgba(255,255,255,0)",
				strokeColor : "rgba(%d,%d,%d,1)",
				pointColor : "rgba(%d,%d,%d,1)",
				data : %s
			},
    """ % (rgbColor[0],rgbColor[1],rgbColor[2],rgbColor[0],rgbColor[1],rgbColor[2],sData)
    return sRet,sLabels
        
def get_plot_html_page(arr,aTimelineForXAxis=None):
    sLabels = ""
    ds = ""
    legende = "<p>Legende:<br>"
    for sName,aColor,data in arr:
        sData,sLabels = create_dataset(data,aColor,aTimelineForXAxis)
        ds += sData
        legende += ('<font color="#%s%s%s">' % (_hex(aColor[0]),_hex(aColor[1]),_hex(aColor[2]))) + sName + "</font><br>"
    s = """
<!doctype html>
<html>
	<head>
		<title>Measurement Plot</title>
		<script src="Chart.min.js"></script>
		<meta name="viewport" content="initial-scale = 1, user-scalable = no">
		<style>
			canvas{
			}
		</style>
	</head>
	<body>
        <a href="view.html">Main Page<a/>
        <p>
		<canvas id="canvas" height="450" width="600"></canvas>
	<script>
		var lineChartData = {
			labels : %s,
			datasets : [
				%s
			]			
		}
        var lineOptions = {
            animation: false,
            bezierCurve: false,
            pointDot: false,
            scaleShowGridLines: false,
            scaleLineColor : "rgba(0,0,0,1)",
        };
	var myLine = new Chart(document.getElementById("canvas").getContext("2d")).Line(lineChartData,lineOptions);
	</script>
    %s
	</body>
</html>
    """ % (sLabels,ds,legende)
    return s

def get_open_closed_none_form(name,value):
    if value==None:
        openChecked = ""
        stopChecked = ""
        closeChecked = ""
        noneChecked = "checked"
    elif value==1:
        openChecked = "checked"
        stopChecked = ""
        closeChecked = ""
        noneChecked = ""
    elif value==-1:
        openChecked = ""
        stopChecked = ""
        closeChecked = "checked"
        noneChecked = ""
    else:
        openChecked = ""
        stopChecked = "checked"
        closeChecked = ""
        noneChecked = ""
    sForm = """
<form action="http://127.0.0.1:%s/help.html" method="get">
    <input type="radio" name="value" value="open" onclick="JSRequest('%s',1)" %s> Open
    <input type="radio" name="value" value="stop" onclick="JSRequest('%s',0)" %s> Close
    <input type="radio" name="value" value="close" onclick="JSRequest('%s',-1)" %s> Stop
    <input type="@radio" name="value" value="none" onclick="JSRequest('%s',null)" %s> None
</form>""" % (g_http_port_no,name,openChecked,name,stopChecked,name,closeChecked,name,noneChecked)
    return sForm

def get_on_off_form(name,value):
    if value==None:
        onChecked = ""
        offChecked = ""
        noneChecked = "checked"
    elif value==1:
        offChecked = ""
        onChecked = "checked"
        noneChecked = ""
    else:
        onChecked = ""
        offChecked = "checked"
        noneChecked = ""
    sForm = """
<form action="http://127.0.0.1:%s/help.html" method="get">
    <input type="radio" name="value" value="on" onclick="JSRequest('%s',1)" %s> On
    <input type="radio" name="value" value="off" onclick="JSRequest('%s',0)" %s> Off
    <input type="radio" name="value" value="none" onclick="JSRequest('%s',null)" %s> None
</form>""" % (g_http_port_no,name,onChecked,name,offChecked,name,noneChecked)
    return sForm
        
def get_stop_form():
    sForm = """
<form action="stop" method="get">
      <input type="submit" value=" Stop Control ">      
</form>"""
    return sForm        

def strip_root(name):
    if name.startswith("/"):
        return name[1:]
    else:
        return name
        
def can_handle_as_file(page):
    return os.path.exists(strip_root(page))
    
def handle_as_file(page):
    return open(strip_root(page),"r").read()

# *************************************************************************

class MyHTTPRequestHandler(SimpleHTTPRequestHandler):

    """ 
        Request handler for heating control application.
    """

    server_version = "MyHTTP/" + __version__
        
    def process_data(self,bHtml,data):
        if bHtml:
            contentTyp = "text/html"
        else:
            contentTyp = "text/plain"
        self.send_header("Content-type", contentTyp)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Last-Modified", self.date_time_string(None))
        self.end_headers()      
        if  sys.version_info.major==3:
            ret = BytesIO(bytes(data, 'latin-1'))
        else:
            ret = StringIO(data)
        return ret
                      
    def send_head(self):
        """
            Overwrite implementation of base class.
            Insert behaviour for heating control application.
            All other request will be handled by the base class.
        """
        print( ">>>>>><<<<<",self.path )
        path = self.path
        if self.aHeatingControlHttpServer!=None:
            bHtml,data,ok = self.aHeatingControlHttpServer.process_html_request(path)
        elif self.aBluetoothClient!=None:
            bHtml,data,ok = self.aBluetoothClient.process_html_request(path)
        else:
            ok = False
        if ok:
            self.send_response(200)                
            return self.process_data(bHtml=bHtml,data=data)
        else:
            # delegate implementation to base class
            return SimpleHTTPRequestHandler.send_head(self)

# *************************************************************************

class HeatingControlHTTPServer:

    """ 
        Request handler for heating control application.
    """

    server_version = "MyHTTP/" + __version__
    
    TIMEOUT_CONTROL_PROCESS = 60    # seconds
        
    def __init__(self,qWrite,qRead,fcnStop):
        self.mapCmds = { 'STOP'            : self.cmd_stop,
                         'EXCEPTION'       : self.cmd_exception,
                         'WRITE'           : self.cmd_write,
                         'READ'            : self.cmd_read,
                         'HISTORY'         : self.cmd_history,
                         'PLOT'            : self.cmd_plot,
                         'READ_PLOT'       : self.cmd_read_plot,
                         'QUERY_PLOT_DATA' : self.cmd_query_plot_data,
                         'CTRL_VERSION'    : self.cmd_control_version,
                         'SRV_VERSION'     : self.cmd_server_version,
# TODO: 'BOARD_VERSION'                         
                         'RESTART'         : self.cmd_restart,
                         'HELP'            : self.cmd_help,
                         'HTML'            : self.cmd_html_view,                         
                         'SIMPLE'          : self.cmd_html_view,                         
                         'VIEW'            : lambda v : self.cmd_html_view(v,bExtended=True),
                         'EXTENDED'        : lambda v : self.cmd_html_view(v,bExtended=True),
                       }
        self.aQueueWrite = qWrite
        self.aQueueRead = qRead
        self.fcnStop = fcnStop
                                                                            
    def process_html_request(self,path):
        """
            Overwrite implementation of base class.
            Insert behaviour for heating control application.
            All other request will be handled by the base class.
        """
        # map request to this server or index.html to html-view of control  
        if path=="/" or path.lower()=="/index.html":
            path = "/html.html"
        page,page_cmd,cmd,args = self._check_path(path)
        print( "process_html_request:",page,page_cmd,cmd,args )
        #import traceback
        #traceback.print_stack()
        if cmd.upper() in self.mapCmds or page_cmd.upper() in self.mapCmds:
            bHtml,data = self._get_data(page,page_cmd,cmd,args)
            return bHtml,data,True
        else:
            if can_handle_as_file(page):
                return False,handle_as_file(page),True
            else:    
                return False,"ERROR: "+page+" not found !",False
    
    def cmd_stop(self,args=None):
        sTag = "DATA_STOP"
        self.aQueueWrite.put("STOP")
        if self.fcnStop:
            self.fcnStop.do_stop(True)
        return sTag,""
            
    def cmd_exception(self,args=None):
        sTag = "DATA_EXCEPTION"
        self.aQueueWrite.put("EXCEPTION")
        return sTag,""
        
    def cmd_write(self,args=None):
        sTag = "DATA_WRITE"             # Bugfix 4.11.2010: entfernt streifen in aktuellem Plot wenn ein Button/Checkbox manuell aktiviert wird --> write-cmd beeinflusste Plot !
        self.aQueueWrite.put("WRITE")
        self.aQueueWrite.put(args)
        try:
            data = self.aQueueRead.get(True,self.TIMEOUT_CONTROL_PROCESS)
        except Exception as aExc:
            print( "EXCEPTION WRITE",aExc )
        return sTag,""

    def cmd_read(self,args=None):
        sTag = "DATA"
        s = ""
        data = self._read_data()
        for key in data:
            # if any key in the result set is equal to RESTART --> send RESTART back to GUI for rebot !
            if key=='RESTART':
                return "RESTART","now"
            s += str(key)
            s += "="
            s += str(data[key])
            s += "\n"
        return sTag,s

    def cmd_history(self,args=None):
        # http://127.0.0.1:8000/history?name=SOLAR_KVLF
        sTag = "HISTORY"
        self.aQueueWrite.put("HISTORY")
        self.aQueueWrite.put(args)
        try:
            data = self.aQueueRead.get(True,self.TIMEOUT_CONTROL_PROCESS)
        except Exception as aExc:
            print( "EXCEPTION HISTORY",aExc )
        return sTag,str(data)

    def _get_plot_data(self,sName):
        sTag = "HISTORY"
        aArgs = { "name" : sName }
        self.aQueueWrite.put("HISTORY")
        self.aQueueWrite.put(aArgs)
        data = []
        try:
            data = self.aQueueRead.get(True,self.TIMEOUT_CONTROL_PROCESS)
        except Exception as aExc:
            print( "EXCEPTION PLOT_DATA",aExc )
        return data        
        
    def cmd_plot(self,args=None):
        sTag = "HTML"
        arr = []
        aTimelineForXAxis = self._get_plot_data("TIMELINE")
        #for test: aTimelineForXAxis = [datetime.time(12 + i / 60,i % 60,0) for i in range(240)]
        for sName,aColor in [("HEAT_CREATOR",(255,0,0)), 
                             ("SOLAR_KVLF",(0,255,0)),
                             ("SOLAR_SLVF", (0,140,0)),
                             ("WARM_WATER", (255,0,255)),
                             ("BUFFER1", (180,0,0)), 
                             ("BUFFER2", (128,0,0)),
                             ("OUTDOOR", (0,0,255)), 
                             ("OUTGOING_AIR", (0,255,255)), 
                             ("CONVERTER", (150,28,170)), 
                             ("MIXER_HEATING", (255,128,0)), 
                             ("ROOM", (164,64,0)), 
                             #("TESTSENSOR_PT1000", (128,128,0))
                             ]:
            data = self._get_plot_data(sName)
            arr.append((sName,aColor,data))
        s = get_plot_html_page(arr,aTimelineForXAxis)
        return sTag,s

    def cmd_read_plot(self,args=None):
        sDate = args.get("DATE_START","01.01.2010")
        iLenInDays = int(args.get("DATE_LENGTH",1))
        lstColDescr = []                                                #["SOLAR_KVLF","SOLAR_SLVF"]
        lstColDescr.append(args.get("COL_NAME_1","SOLAR_KVLF"))   
        lstColDescr.append(args.get("COL_NAME_2","SOLAR_SVLF"))   
        lstColDescr.append(args.get("COL_NAME_3","OUTDOOR"))   
        import view_temp
        data = view_temp.get_plot_data_str(sDate,iLenInDays,lstColDescr)    # --> ( [x], [ [y1], [y2], ... ] )
        sTag = "PLOT"
        s = ""
        newData = [data[0]]
        for i in range(len(data[1])):
            newData.append(data[1][i])
        data = newData
        if len(data)>0:
            for i in range(len(data[0])):
                s += str(data[0][i])
                s += ";"
                for j in range(1,len(data)):
                    s += str(data[j][i])
                    s += ";"
                s = s[:-1]
                s += "\n"
        return sTag,s
        
    def cmd_query_plot_data(self,args=None):
        import view_temp
        data = view_temp.query_available_plot_data()
        sTag = "QUERY"
        s = ""
        for e in data:
            s += str(e)
            s += "\n"
        return sTag,s
                
    def cmd_control_version(self,args=None):
        sTag = "CTRL_VERSION"
        s = ""
        self.aQueueWrite.put("VERSION")
        try:
            data = self.aQueueRead.get(True,self.TIMEOUT_CONTROL_PROCESS)
            s = data+"\n"
        except Exception as aExc:
            print( "EXCEPTION VERSION",aExc )
        return sTag,s

    def cmd_server_version(self,args=None):
        sTag = "SRV_VERSION"
        return sTag,"Server Process Version: "+__version__+" from "+__date__+"\n"
        
    def cmd_restart(self,args=None):
        sTag = "RESTART"
        return sTag,"NOW"
                
    def cmd_help(self,args=None):
        sTag = "HELP"
        return sTag,"Help:\navailable commands:\n"+self._make_help_string()
        
    def cmd_html_view(self,args=None,bExtended=False):
        sTag = "HTML"
        sScript = """<script type="text/javascript">
// reload the page ...
//window.setTimeout(function () { location.reload(1); }, 5000);

function JSRequest(name,val) {
  var xhReq = new XMLHttpRequest();
  if( val==null )
    xhReq.open("GET", "write?"+name+"=None", false);
  else  
    xhReq.open("GET", "write?"+name+"="+val, false);
  xhReq.send();
  var serverResponse = xhReq.responseText;
  //alert(serverResponse);
  //ggf. delay falls wert setzten langsamer als update ist !!! ...
  window.location.reload();
}
</script>
        """
        s = "<html>"
        #s += '<meta http-equiv="refresh" content="3" />'
        s += "<head>"
        ##s += '<script src="Chart.min.js"></script>'
        s += "</head>"
        ##s += '<canvas id="myCanvas" height="450" width="600"></canvas>'
        s += sScript
        #s += '<body onload="drawCanvas()">'
        s += '<a href="plot.html">Measurement Plots<a/>'
        if bExtended:
            data = self._read_data_extended()
        else:
            data = self._read_data()
        sm = "<table>"
        sm += "<tr><th>Name</th><th>Value</th><th>Action</th></tr>"
        smv = "<table>"
        smv += "<tr><th>Name</th><th>Value</th><th>Action</th></tr>"
        si = "<table>"
        si += "<tr><th>Name</th><th>Value</th></tr>"
        sv = "<table>"
        if bExtended:
            sv += "<tr><th>Name</th><th>Value [°C]</th><th>Std Dev</th></tr>"
        else:
            sv += "<tr><th>Name</th><th>Value [°C]</th></tr>"
        ss = "<table>"
        ss += "<tr><th>Name</th><th>Value</th></tr>"
        sc = "<table>"
        sc += "<tr><th>Name</th><th>Value</th></tr>"
        so = "<table>"
        so += "<tr><th>Name</th><th>Value [s]</th><th>Value [h]</th><th>Value [d]</th><th>Value Day Histogram [h]</th></tr>"
        sp = "<table>"
        sp += "<tr><th>Name</th><th>Value</th></tr>"
        for e in data:
            if bExtended:
                if type(data[e])==tuple:
                    val = data[e][0]
                    stddev = data[e][1]
                else:
                    val = data[e]
                    stddev = -1.0
            else:
                val = data[e]
                stddev = 0.0
            if e in ["MASTER_ID","TICK_COUNTER","ARDURINO_COUNT","WATCHDOG","ACT_TICK","TIMELINE","ENABLE_HEATING_MOTOR","ENABLE_HEATING_MOTOR_NIGHT","ENABLE_HEATING_MOTOR_VALUE","ENABLE_HEATING_MOTOR_DATE_AND_VALUE","ENABLE_HEATING_MOTOR_DATE","ENABLE_DATE_HEATING_MOTOR","ENABLE_MIXER_CLOSE","ENABLE_ANTI_FIXING_SWITCH","ENABLE_MIXER_MOVEMENT","ENABLE_HEATING_MOTOR_ALL_INPUTS","ENABLE_HEAT_PUMP","NOT_MANUAL_SWITCH_HEATING_DATE","TEMP_MEAS","RELAIS_MEAS"]:
                sp += "<tr>"
                sp += "<td>"+e+"</td>"
                sp += "<td>"+str(val)+"</td>"
                sp += "</tr>"
            elif e in ["HEATPUMP_CONTROL_SWITCH_OFF_AIR_TEMP","MIXER_HEATING_DES","HEATPUMP_CONTROL_SWITCH_OFF_TEMP","MIXER_HEATING_DES_SWITCH_ON_TEMP","HEATING_CONTROL_SWITCH_ON_TEMP", "HEATPUMP_CONTROL_SWITCH_ON_TEMP", "SWITCH_MIXER_HEATING_CURRENT_OPEN_COUNT", "MIXER_HEATING_DES_CURRENT_DESIRED_TEMPERATURE", "MIXER_HEATING_DES_CURRENT_MAX_TEMPERATURE"]:
                smv += "<tr>"
                smv += "<td>"+e+"</td>"
                smv += "<td>"+str(val)+"</td>"
                smv += "</tr>"
            elif e.count("_INTERVALS")>0:
                si += "<tr>"
                si += "<td>"+e+"</td>"
                si += "<td>"+str(val)+"</td>"
                si += "</tr>"
            elif e.startswith("MANUAL_SWITCH_MIXER_HEATING"):
                sm += "<tr>"
                sm += "<td>"+e+"</td>"
                sm += "<td>"+str(val)+"</td>"
                sm += "<td>"+get_open_closed_none_form(e,str_to_value(val))+"</td>"
                sm += "</tr>"
#            elif e.startswith("MANUAL_SWITCH_HEATING_ONLY_NIGHT"):
#                sm += "<tr>"
#                sm += "<td>"+e+"</td>"
#                sm += "<td>"+str(val)+"</td>"
#                sm += "<td>This value is calculated only !</td>"
#                sm += "</tr>"
            elif e.startswith("MANUAL"):
                sm += "<tr>"
                sm += "<td>"+e+"</td>"
                sm += "<td>"+str(val)+"</td>"
                sm += "<td>"+get_on_off_form(e,str_to_value(val))+"</td>"
                sm += "</tr>"
            elif e.startswith("SWITCH"):
                ss += "<tr>"
                ss += "<td>"+e+"</td>"
                ss += "<td>"+str(val)+"</td>"
                ss += "</tr>"
            elif e.endswith("CONTROL"):
                sc += "<tr>"
                sc += "<td>"+e+"</td>"
                sc += "<td>"+str(val)+"</td>"
                sc += "</tr>"
            elif e.startswith("OPERATING_HOURS"):
                so += "<tr>"
                so += "<td>"+e+"</td>"
                _val = val[0]
                so += "<td>"+str(round(_val, 1))+"</td>"
                so += "<td>"+str(round(float(_val)/3600.0, 2))+"</td>"
                so += "<td>"+str(round(float(_val)/3600.0/24.0, 2))+"</td>"
                delta_histogram = []
                max = len(val[1])
                for i in range(max):  # is list of operating seconds counter
                    if i+1 < max:
                        value = val[1][i+1] - val[1][i]
                        delta_histogram.append(str(round(float(value / 60.0 / 60.0), 2)))  # convert seconds into hours
                so += "<td>"+str(delta_histogram)+"</td>"
                so += "</tr>"
            else:
                sv += "<tr>"
                sv += "<td>"+e+"</td>"
                sv += "<td>"+str(val)+"</td>"
                if bExtended:
                    sv += "<td>"+str(stddev)+"</td>"
                sv += "</tr>"
        sm += "</table>"
        sv += "</table>"
        ss += "</table>"
        sc += "</table>"
        so += "</table>"
        sp += "</table>"
        si += "</table>"
        smv += "</table>"   
        s += "<p><b>Current Control Values:</b><p>"
        s += sm
        s += "<p><b>Current Temperature Values [°C]:</b><p>"
        s += sv
        s += "<p><b>Some Control Values:</b><p>"
        s += smv
        s += "<p><b>Current Switch Values:</b><p>"
        s += ss
        s += "<p><b>Current Control Values:</b><p>"
        s += sc
        s += "<p><b>Current Operating Hours Values [s]:</b><p>"
        s += so
        s += "<p><b>Some Other Values:</b><p>"
        s += sp
        s += "<p><b>Intervalls:</b><p>"
        s += si
        s += "<p>" 
# TODO --> nur im Testbetrieb stop ermoeglichen        
        s += get_stop_form()
        s += "</body></html>"
        return sTag,s
                                
    def _read_data(self):
        data = {}
        self.aQueueWrite.put("READ")
        try:
            data = self.aQueueRead.get(True,self.TIMEOUT_CONTROL_PROCESS)
        except Exception as aExc:
            print( "EXCEPTION READ",aExc )
        return data

    def _read_data_extended(self):
        data = {}
        self.aQueueWrite.put("READ_EXTENDED")
        try:
            data = self.aQueueRead.get(True,self.TIMEOUT_CONTROL_PROCESS)
        except Exception as aExc:
            print( "EXCEPTION READ_EXTENDED",aExc )
        return data

    def _get_data(self,page,page_cmd,cmd,args):
        data = "?"
        s = ""
        cmd = cmd.upper()
        if self.aQueueRead!=None and self.aQueueWrite!=None:
            if cmd in self.mapCmds:
                sTag,s = self.mapCmds[cmd](args)
            elif page_cmd in self.mapCmds:
                sTag,s = self.mapCmds[page_cmd](args)
            else:
                sTag,s = "DATA_UNKNOWN",""
        else:
            s = "NO_QUEUE_TO_CONTROL_AVAILABLE"
            sTag = "HTML"
        #return "Command=%s %s %s %s\n\n<%s>\n%s</%s>" % (cmd,self.aQueueWrite,self.aQueueRead,data,sTag,s,sTag)
        if sTag=="HTML":
            return (True,s)
        else:
            return (False,"<%s>\n%s</%s>" % (sTag,s,sTag))
    
    def _make_help_string(self):
        s = ""
        for key in self.mapCmds:
            s += "* "+key+"\n"
        return s
       
    def _command_for_page(self, page):
        page = page.upper()
        page = page.strip("/")
        if page.endswith(".HTML"):
            page = page[:-5]
        return page
        
    def _check_path(self, path):
        """
            Translate the request path into processable data:
            
            http://127.0.0.1:8000/index.html;command?arg1=val1;arg2=val2#ref
            
            (page,command,{arguments_key:arguments_value})
        """
        path = urlparse.urlparse(path)
        args = {}
        page = path[2]      # --> /index.html
        cmd = path[3]
        temp = path[4]
        temp = temp.split(";")
        for e in temp:
            v = e.split("=")
            if len(v)>1:
                args[v[0]]=v[1]
        return (page,self._command_for_page(page),cmd,args)

# **********************************************************************
class StopProcess:
    
    def __init__(self):
        self.bStop = False
        
    def do_stop(self,bValue=True):
        self.bStop = bValue
        
    def is_stop(self):
        return self.bStop
        
    def __call__(self):
        return self.is_stop()

# *************************************************************************

STOP_SEQUENCE = "<.done.>" 

class BluetoothServer:
    """
        Forwards HTTP requests via Bluetooth
    """
    
    PORT = DEFAULT_PORT
    BACKLOG = 1

    def __init__(self,heatingControlHttpServer):
        self.bStop = False
        self.aHeatingControlHttpServer = heatingControlHttpServer
        self.server_sock = None
        self.aThread = threading.Thread(None,self.run)
        self.aThread.start()
        
    def stop(self):
        print( "========> CALLING STOP stop" )
        if self.server_sock!=None:
            self.server_sock.close()
        self.bStop = True
    
    def communicate_with_client(self,client,address):
        bContinue = True
        while bContinue:    
            try:
                req = client.recv(4096) 
                #print "request:",type(req)
                #print len(req),req
                if self.aHeatingControlHttpServer!=None:
                    bHtml,data,ok = self.aHeatingControlHttpServer.process_html_request(req)
                    #print "via bluetooth",bHtml,data,ok
                    if True: #ok:
                        client.sendall(data+STOP_SEQUENCE)
                else:
                    print( "NO delegate to server set !" )
                    client.sendall("answer:"+req+STOP_SEQUENCE)
                if req=='stop':
                    self.stop()
                #client.flush()
                #bContinue = False                
#            data = get_data_from_controla(req)
 #           write_response(client,data)  
            except Exception as exc:
                print( "Exception:",exc )
                bContinue = False
        #print "close socket"
        time.sleep(1)
        client.close()
        
    def run(self):
        self.server_sock = BluetoothSocket(bluetooth.RFCOMM)
        self.server_sock.bind(("",self.PORT))
        self.server_sock.listen(self.BACKLOG)
        #self.server_sock.settimeout(1)
        #print help(self.server_sock.settimeout)
        #print help(self.server_sock.setblocking)
        #self.server_sock.setblocking(True)
    
        while not self.bStop:
            print( "trying to accept..." )
            client_sock,address = self.server_sock.accept()
            print( "accepted:",client_sock,address )
            self.communicate_with_client(client_sock,address)
            time.sleep(0.5)
        print( "BLUETOOTH server stoped" )
        self.server_sock.close()
        
# *************************************************************************

class BluetoothClient:

    PORT = 3

    def __init__(self,address):
        self.address = address
        self.socket = BluetoothSocket(bluetooth.RFCOMM)
        ok = self.socket.connect((self.address,self.PORT))
        
    def write(self,data):
        ret = self.socket.sendall(data)
        #print "send:",data
        #self.socket.flush()
        #print "flushed"
        return ret
        
    def read(self):
        COUNT = 512
        ret = self.socket.recv(COUNT)
        bContinue = ret!=None and not ret.endswith(STOP_SEQUENCE)
        #print "read",ret
        while bContinue:
            data = self.socket.recv(COUNT)
            #print "continue",data,len(data)
            ret += data
            if ret.endswith(STOP_SEQUENCE):
                bContinue = False
        #print "read done",bContinue
        if ret.endswith(STOP_SEQUENCE):
            return ret[:-len(STOP_SEQUENCE)]
        return ret
        
    def write_and_read(self,data):
        self.write(data)
        return self.read()

    def close(self):
        #print "close socket"
        self.socket.close()
        
    def process_html_request(self,req):
        data = self.write_and_read(req)
        #self.close()
        return (True,data,True)
        
# *************************************************************************

def run_server(port,
               qWrite,
               qRead,
               bIsTest = False,
               bUseHttpDefaultPort = False,
               HandlerClass = MyHTTPRequestHandler,
               ServerClass = HTTPServer, 
               protocol = "HTTP/1.0",
               bBluetoothClient = False):
    global g_http_port_no
    g_http_port_no = port

    server_address = ('', port)
    #server_address = ('127.0.0.1', port)

    fcnStop = StopProcess()

    if not g_bFoundBluetooth:                        
        print( "Warning: BLUETOOTH not found!" )

    bluetoothClient = None
    bluetoothServer = None
    if bBluetoothClient:
        heatingControlHttpServer = None
        if g_bFoundBluetooth:                        
            bluetoothClient = BluetoothClient('00:1B:DC:0F:AA:02')
            print( "Info: BLUETOOTH was found !",bluetoothClient )
    else:
        heatingControlHttpServer = HeatingControlHTTPServer(qWrite,qRead,fcnStop)
        if g_bFoundBluetooth:                        
            bluetoothServer = BluetoothServer(heatingControlHttpServer)
            print( "Info: BLUETOOTH was found !",bluetoothServer )
    HandlerClass.protocol_version = protocol
    HandlerClass.aHeatingControlHttpServer = heatingControlHttpServer
    HandlerClass.aBluetoothClient = bluetoothClient
    httpd = ServerClass(server_address, HandlerClass)

# TODO heatingControlHttpServer auch an BluetoothServer uebergeben...

    sa = httpd.socket.getsockname()
    print( "Serving HTTP on", sa[0], "port", sa[1], "..." )
    httpd.serve_forever()
#    while not fcnStop():
#        print "->",
#        httpd.handle_request()  
#        print "<-",
    return True  

def run_client():
    client = BluetoothClient('00:1B:DC:0F:AA:02')
    client.write("help")
    data = client.read()
    print( "read:",data )

# *************************************************************************

if __name__ == '__main__':
    import sys
    if '-t' in sys.argv:
        # test client only !
        run_client()
    else:
        print( "starting server" )
        #run_server(None,None,lambda v1,v2,v3 : False,bBluetoothClient='-c' in sys.argv)
        run_server(None,None,bBluetoothClient='-c' in sys.argv)
        print( "done." )
    