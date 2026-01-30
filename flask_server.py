from flask import Flask, render_template_string, request, Response
import paho.mqtt.client as mqtt

import json
import queue
#import uuid
#import time

import simplesrv

g_http_port_no = 5000

# Flask App
app = Flask(__name__)

# Globale Variable für Temperatur
temperature = "Noch keine Daten"

g_all_data = {}

def remove_path_prefixes(s, prefixes):
    for prefix in prefixes:
        if s.startswith(prefix):
            return s[len(prefix):]
    return s

def check_type(s):
    """
       Prüft, ob der String in einen Integer oder Float umgewandelt werden kann.
    """
    try:
        if s.lower() == 'none':
            return None
        if '.' in s:
            return float(s)
        else:
            return int(s)
    except:
        return s

# MQTT Callback: wenn Nachricht empfangen
def on_message(client, userdata, msg):
    if msg.topic == 'heating/temperature/outdoor':
        global temperature
        temperature = check_type(msg.payload.decode("utf-8"))
    global g_all_data
    if msg.topic == 'heating/tick_no':
        g_all_data['ACT_TICK'] = check_type(msg.payload.decode("utf-8"))
        #print("Tick No:", g_all_data['ACT_TICK'])
    else:
        g_all_data[remove_path_prefixes(msg.topic, ["heating/temperature/", "heating/manual_switch/", "heating/switch/", "heating/control/", "heating/operating_hours/", "heating/intervals/"]).upper()] = check_type(msg.payload.decode("utf-8"))
#    if msg.topic == 'heating/tick_no':
#        print(g_all_data)
    if msg.topic == 'heating/history/response':
        history_response_data = json.loads(msg.payload.decode("utf-8"))
        g_all_data['HISTORY_ALL'] = history_response_data
        response_queue.put("HISTORY_RECEIVED")
        
    # if msg.topic.startswith("heating/history/response/"):
    #     history_response_name = msg.topic[len("heating/history/response/"):]
    #     history_response_data = json.loads(msg.payload.decode("utf-8"))
    #     g_all_data[f'HISTORY_{history_response_name.upper()}'] = history_response_data

def trigger_history_plot_request():    
    signal_names = ['HEAT_CREATOR', 'SOLAR_KVLF', 'SOLAR_SLVF', 'WARM_WATER', 'BUFFER1', 'BUFFER2', 'OUTDOOR', 'OUTGOING_AIR', 'CONVERTER', 'MIXER_HEATING', 'ROOM']  # 'TIMELINE' wird automatisch hinzugefügt
    mqtt_client.publish("heating/history/request", json.dumps(signal_names), qos=2, retain=False)
    # TODO: asynchrone Anfrage
    #for signal_name in signal_names:
    #    mqtt_client.publish(f"heating/history/request/{signal_name}", "", qos=2, retain=False)

# Callback, wenn Verbindung hergestellt ist
def on_connect(client, userdata, flags, rc):
    print("Verbunden mit Code:", rc)
    # Bei erfolgreicher Verbindung ein Topic abonnieren
    client.subscribe("heating/tick_no")
    client.subscribe("heating/temperature/#")
    # client.subscribe("heating/temperature/temp_meas")
    # client.subscribe("heating/temperature/light1")
    # client.subscribe("heating/temperature/light2")
    # client.subscribe("heating/temperature/solar_kvlf")
    # client.subscribe("heating/temperature/solar_slvf")
    # client.subscribe("heating/temperature/outdoor")
    # client.subscribe("heating/temperature/mixer_heating")
    # client.subscribe("heating/temperature/buffer1")
    # client.subscribe("heating/temperature/buffer2")
    # client.subscribe("heating/temperature/outgoing_air")
    # client.subscribe("heating/temperature/warm_water")
    # client.subscribe("heating/temperature/heat_creator")
    # client.subscribe("heating/temperature/converter")
    # client.subscribe("heating/temperature/room")
    # client.subscribe("heating/temperature/testsensor_pt1000")
    # client.subscribe("heating/temperature/esp32_http_temp")
    client.subscribe("heating/manual_switch/#")
    # client.subscribe("heating/manual_switch/manual_switch_motor_solar")
    # client.subscribe("heating/manual_switch/manual_switch_motor_heating")
    # client.subscribe("heating/manual_switch/manual_switch_heating_only_night")
    # client.subscribe("heating/manual_switch/manual_switch_heating_date")
    # client.subscribe("heating/manual_switch/manual_switch_max_temp_control")
    # client.subscribe("heating/manual_switch/manual_switch_mixer_heating")
    # client.subscribe("heating/manual_switch/manual_switch_heatpump")
    # client.subscribe("heating/manual_switch/manual_switch_ventilation")
    # client.subscribe("heating/manual_switch/manual_switch_heatpump_solar_valve")
    # client.subscribe("heating/manual_switch/manual_switch_booster")
    # client.subscribe("heating/manual_switch/manual_switch_heatpump_only_with_pv")
    # client.subscribe("heating/switch/switch_motor_solar")
    # client.subscribe("heating/switch/switch_motor_heating")
    # client.subscribe("heating/switch/switch_mixer_heating")
    # client.subscribe("heating/switch/switch_heatpump")
    # client.subscribe("heating/switch/switch_ventilation")
    # client.subscribe("heating/switch/switch_heatpump_solar_valve")
    # client.subscribe("heating/switch/switch_booster")
    client.subscribe("heating/switch/#")
    # 'SWITCH_MOTOR_SOLAR','SWITCH_MOTOR_HEATING','SWITCH_MIXER_HEATING','SWITCH_HEATPUMP','SWITCH_VENTILATION','SWITCH_HEATPUMP_SOLAR_VALVE','SWITCH_BOOSTER'
    client.subscribe("heating/control/#")
    client.subscribe("heating/operating_hours/#")
    client.subscribe("heating/intervals/#")
    client.subscribe("heating/history/response/#")
    
# MQTT Setup
mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message
mqtt_client.on_connect = on_connect
mqtt_client.connect("127.0.0.1", 1883, 60)   # Broker-Adresse anpassen
#mqtt_client.connect("192.168.178.40", 1883, 60)   # Broker-Adresse anpassen
mqtt_client.loop_start()

response_queue = queue.Queue()

_helper = simplesrv.HeatingControlHTTPServer(None,None,None)

# Flask Route
@app.route("/temp")
def index():
    html = """
    <html>
      <head><title>Temperaturanzeige</title></head>
      <body>
        <h1>Aktuelle Temperatur</h1>
        <p style="font-size:2em;">{{ temp }} °C</p>
      </body>
    </html>
    """
    return render_template_string(html, temp=temperature)

@app.route("/")
@app.route("/view")
@app.route("/view.html")
@app.route("/index.html")
def view():
    return simplesrv.get_view(g_all_data, bExtended=False)

@app.route("/plot")
@app.route("/plot.html")
def plot():
    trigger_history_plot_request()
    
    try: 
        # Auf Antwort warten (max 20 Sekunden) 
        result = response_queue.get(timeout=20)
    except queue.Empty: 
        return "Timeout – no responce recieved", 504
    
    all_y_data = g_all_data['HISTORY_ALL'][1]
    timeline_data = g_all_data['HISTORY_ALL'][0]
    return simplesrv.get_plot_html_page(all_y_data, timeline_data)

@app.route("/help")
def help():
    _sTag, s =_helper.cmd_help()
    return Response(s, mimetype="text/plain")

@app.route("/write", methods=["GET"])
def write():
    # Query-Parameter auslesen
    keys = list(request.args.keys())
    print("+++++++ WRITE called with keys =", keys)
    if len(keys) > 0:
        key = keys[0]
        value = request.args.get(key)
        new_value = check_type(value)
# TODO -> hier ggf. einen postfix an den topic namen anhängen, damit in der heitzungsloop nicht so häufig Daten über MQTT geschrieben und verarbeitet werden 
        mqtt_client.publish(f"heating/manual_switch/{key.lower()}", str(new_value), qos=2, retain=True)
        print(f"**** Published {new_value} to {key} ****")
        return f"Published {new_value} to heating/manual_switch/{key.lower()}"
    else:
        return "No parameters provided", 400
        
# <HELP>
# Help:
# available commands:
# * STOP
# * EXCEPTION         -
# * WRITE             for: control
# * READ              for: exporter
# * HISTORY           -
# * PLOT              needed
# * READ_PLOT
# * QUERY_PLOT_DATA
# * CTRL_VERSION      ok
# * SRV_VERSION       ok
# * RESTART           -
# * HELP              ok
# * HTML              == VIEW
# * SIMPLE            == VIEW
# * VIEW              needed
# * EXTENDED          == VIEW
# </HELP>

if __name__ == "__main__":
    # TODO: verwende Gunicorn oder ähnliches für den produktiven Einsatz !
    app.run(host="0.0.0.0", port=g_http_port_no)
