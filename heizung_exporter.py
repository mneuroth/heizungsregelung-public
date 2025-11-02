"""
    Heizungsregelung Exporter

    Started: 5.4.2023

    see: https://trstringer.com/quick-and-easy-prometheus-exporter/

    Possible parameters:

      Environment variables:
      - POLLING_INTERVAL_SECONDS  -> default value: 5
      - APP_PORT                  -> default value: 42424
      - EXPORTER_PORT             -> default value: 9110

"""

import os
import time
#import requests
import socket
import selectors
import traceback
import json

import libclient

from prometheus_client import start_http_server, Gauge, Enum

__version__ = "1.0.3"
__date__    = "9.11.2024"

def create_request(action, value):
    return dict(
        type="text/json",
        encoding="utf-8",
        content=dict(action=action, value=value),
    )

def start_connection(sel, host, port, request):
    addr = (host, port)
    #print(f"Starting connection to {addr}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setblocking(False)
    sock.connect_ex(addr)
    events = selectors.EVENT_READ | selectors.EVENT_WRITE
    message = libclient.Message(sel, sock, addr, request)
    sel.register(sock, events, data=message)
    return message

def check_for_none(val):
    if val is None:
        return 0.0
    if isinstance(val, str):
        try:
            return float(val)
        except ValueError:
            return 0.0
    return val


class AppMetrics:
    """
    Representation of Prometheus metrics and loop to fetch and transform
    application metrics into Prometheus metrics.
    """

    def __init__(self, app_port=80, polling_interval_seconds=5):
        self.app_port = app_port
        self.polling_interval_seconds = polling_interval_seconds

        # Prometheus metrics to collect
        self.temp_outdoor = Gauge("temp_outdoor", "Outdoor Temperature")
        self.temp_converter = Gauge("temp_converter", "Converter Temperature")
        self.temp_warm_water = Gauge("temp_warm_water", "Warm Water Temperature")
        self.temp_room = Gauge("temp_room", "Room Temperature")
        self.temp_buffer1 = Gauge("temp_buffer1", "Buffer 1 Temperature")
        self.temp_buffer2 = Gauge("temp_buffer2", "Buffer 2 Temperature")
        self.temp_outgoing_air = Gauge("temp_outgoing_air", "Outgoing Air Temperature")
        self.temp_ingoing_air = Gauge("temp_ingoing_air", "Ingoing Air Temperature")
        self.temp_solar_slvf = Gauge("temp_solar_slvf", "Solar SLVF Temperature")
        self.temp_solar_kvlf = Gauge("temp_solar_kvlfr", "Solar KVLF Temperature")
        self.temp_mixer_heating = Gauge("temp_mixer_heating", "Mixer Heating Temperature")
        self.temp_heat_creator = Gauge("temp_heat_creator", "Heat Creator Temperature")
        self.temp_esp32_http = Gauge("temp_esp32_http", "ESP32 Http Temperature")

        self.switch_motor_solar = Gauge("switch_motor_solar", "Switch Motor Solar")
        self.switch_motor_heating = Gauge("switch_motor_heating", "Switch Motor Heating")
        self.switch_mixer_heating = Gauge("switch_mixer_heating", "Switch Mixer Heating")
        self.switch_heatpump = Gauge("switch_heatpump", "Switch Heatpump")
        self.switch_ventilation = Gauge("switch_ventilation", "Switch Ventilation")
        self.switch_heatpump_solar_valve = Gauge("switch_heatpump_solar_valve", "Switch Heatpump Solar Valve")
        self.switch_booster = Gauge("switch_booster", "Switch Booster")

        self.health = Enum("app_health", "Health", states=["healthy", "unhealthy"])

    def run_metrics_loop(self):
        """Metrics fetching loop"""

        while True:
            self.fetch()
            time.sleep(self.polling_interval_seconds)

    def fetch(self):
        """
        Get metrics from application and refresh Prometheus metrics with
        new values.
        """

        # Fetch raw status data from the application
        #resp = requests.get(url=f"http://localhost:{self.app_port}/status")
        #status_data = resp.json()

        status_data = {}
        status_data = self._read_heizungsregelung_data()

        # for debugging:
        #print(".",)
        #print("TEMPERATURES:", type(status_data), status_data)
        #for k in status_data:
        #    print(k, "->", type(status_data[k]), status_data[k])

        # Update Prometheus metrics with application metrics
        if len(status_data) > 0:
            self.temp_outdoor.set(status_data["OUTDOOR"])
            self.temp_converter.set(status_data["CONVERTER"])
            self.temp_warm_water.set(status_data["WARM_WATER"])
            self.temp_room.set(status_data["ROOM"])
            self.temp_buffer1.set(status_data["BUFFER1"])
            self.temp_buffer2.set(status_data["BUFFER2"])
            self.temp_outgoing_air.set(status_data["OUTGOING_AIR"])
            self.temp_ingoing_air.set(status_data["INGOING_AIR"])   # == temp_board
            self.temp_solar_slvf.set(status_data["SOLAR_SLVF"])
            self.temp_solar_kvlf.set(status_data["SOLAR_KVLF"])
            self.temp_mixer_heating.set(status_data["MIXER_HEATING"])
            self.temp_heat_creator.set(status_data["HEAT_CREATOR"])
            self.temp_esp32_http.set(status_data["ESP32_HTTP_TEMP"])

            self.switch_motor_solar.set(check_for_none(status_data["SWITCH_MOTOR_SOLAR"]))
            self.switch_motor_heating.set(check_for_none(status_data["SWITCH_MOTOR_HEATING"]))
            self.switch_mixer_heating.set(check_for_none(status_data["SWITCH_MIXER_HEATING"]))
            self.switch_heatpump.set(check_for_none(status_data["SWITCH_HEATPUMP"]))
            self.switch_ventilation.set(check_for_none(status_data["SWITCH_VENTILATION"]))
            self.switch_heatpump_solar_valve.set(check_for_none(status_data["SWITCH_HEATPUMP_SOLAR_VALVE"]))
            self.switch_booster.set(check_for_none(status_data["SWITCH_BOOSTER"]))

            #self.health.state(status_data["health"])

# TODO -> falls keine Werte von der Heizungsregelung kommen -> verwende/setze 0 ? 

    def _read_heizungsregelung_data(self):
        """
            Read via tcpip socket the current values of the heating control process.
        """
        host = "localhost"
        sel = selectors.DefaultSelector()
        request = create_request("cmd", "READ")
        message = start_connection(sel, host, self.app_port, request)

        try:
            while True:
                events = sel.select(timeout=1)
                for key, mask in events:
                    message = key.data
                    try:
                        message.process_events(mask)
                    except Exception:
                        print(
                            f"Main: Error: Exception for {message.addr}:\n"
                            f"{traceback.format_exc()}"
                        )
                        message.close()
                # Check for a socket being monitored to continue.
                if not sel.get_map():
                    break
        except KeyboardInterrupt:
            print("Caught keyboard interrupt, exiting")
        finally:
            sel.close()
        ret = {}
        if message.result is not None and len(message.result) > 0:
            ret = json.loads(message.result)
        return ret


def main():
    """Main entry point"""

    polling_interval_seconds = int(os.getenv("POLLING_INTERVAL_SECONDS", "5"))
    app_port = int(os.getenv("APP_PORT", "42424"))
    exporter_port = int(os.getenv("EXPORTER_PORT", "9110"))
    print("Prometheus exporter for heizungsgegelung on port="+str(exporter_port))

    app_metrics = AppMetrics(
        app_port=app_port,
        polling_interval_seconds=polling_interval_seconds
    )
    start_http_server(exporter_port)
    app_metrics.run_metrics_loop()

if __name__ == "__main__":
    main()
