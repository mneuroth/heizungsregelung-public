"""
    PV facility exporter for Grafana.
"""
import time
import datetime
import os
import sys
import threading
import signal

from pv_facility_reader import PV_Facility

from prometheus_client import start_http_server, Gauge, Enum

class AppMetrics:
    """
    Representation of Prometheus metrics and loop to fetch and transform
    application metrics into Prometheus metrics.
    """
    # - tägliche Einspeisung ins Netz                ok -> Delta (1)
    # - täglicher Bezug vom Netz                     ok -> Delta (2)
    # - tägliche PV-Anlage &| Akku ins Haus &| Netz  ok -> Delta (3)
    # - tägliches Akku laden                         ok -> Delta (4)
    # - tägliches Akku entladen                      ok -> Delta (5)
    # - tägliche Bilanz Akku (netto aufgeladen oder entladen)  Delta Akku geladen (4) - Delta Akku entladen (5) = Akku Bilanz (6)
    # - täglicher Verbrauch von PV-Anlage            ?  => Erzeugung (3) - Enspeisung (1) - Akku Bilanz (6) = Verbrauch von PV-Anlage (7)  -> besser: (3) - (1)
    # - täglicher Verbrauch des Hauses               ?  => Bezug vom Netz (2) + Verbrauch von PV-Anlage (7) = Verbrauch Haus (8)           -> besser: (2) + (3) - (1) = (2) + (7)

    # - täglicher gesamt Yield PV-Anlage -> (1) export grid + (3) export house + (4) charge accu
    # - PV-Yield (3) -> zählt auch Akku Endladungen mit

    def __init__(self, polling_interval_seconds=60, is_debugging=False):
        self.polling_interval_seconds = polling_interval_seconds
        self.is_debugging = is_debugging
        self.is_shutdown = False

        # start measuring of pv values in background thread
        self.pv_facility = PV_Facility(is_debugging=is_debugging)
        self.measure_thread = threading.Thread(target=self.pv_facility.pv_run, args=())
        self.measure_thread.start()

        self.current_day = datetime.datetime.now().date()

        # wait for first read of values...
        time.sleep(30)

        # Prometheus metrics to collect

        # (1) pv ==> grid
        self.grid_exported_energy = Gauge("grid_exported_energy", "Grid Exported Energy")
        self.grid_exported_delta_energy = Gauge("grid_exported_delta_energy", "Grid Exported Delta Energy")
        self.grid_exported_delta_power = Gauge("grid_exported_delta_power", "Grid Exported Delta Power")

        # (2) house <= grid
        self.grid_accumulated_energy = Gauge("grid_accumulated_energy", "Grid Accumulated Energy")        
        self.grid_delta_energy = Gauge("grid_delta_energy", "Grid Delta Energy")
        self.grid_delta_power = Gauge("grid_delta_power", "Grid Delta Power")

        # (3) pv &| accu ==> house &| grid
        self.yield_energy = Gauge("yield_energy", "Yield Energy")
        self.yield_delta_energy = Gauge("yield_delta_energy", "Yield Delta Energy")
        self.yield_delta_power = Gauge("yield_delta_power", "Yield Delta Power")

        # (4) storage <== pv
        self.storage_charge_energy = Gauge("storage_charge_energy", "Storage Charge Energy")
        self.storage_charge_delta_energy = Gauge("storage_charge_delta_energy", "Storage Charge Delta Energy")
        self.storage_charge_delta_power = Gauge("storage_charge_delta_power", "Storage Charge Delta Power")

        # (5) storage ==> house
        self.storage_discharge_energy = Gauge("storage_discharge_energy", "Storage Discharge Energy")
        self.storage_discharge_delta_energy = Gauge("storage_discharge_delta_energy", "Storage Discharge Delta Energy")
        self.storage_discharge_delta_power = Gauge("storage_discharge_delta_power", "Storage Discharge Delta Power")

        # (6) storage +/-
        self.storage_balance_delta_energy = Gauge("storage_balance_delta_energy", "Storage Balance Delta Energy")
        self.storage_balance_delta_power = Gauge("storage_balance_delta_power", "Storage Balance Delta Power")

        # (7) PV consumption house
        self.pv_consumed_house_energy = Gauge("pv_consumed_house_energy", "Used PV Consumed House Energy")
        self.pv_consumed_house_delta_energy = Gauge("pv_consumed_house_delta_energy", "Used PV Consumed House Delta Energy")
        self.pv_consumed_house_delta_power = Gauge("pv_consumed_house_delta_power", "Used PV Consumed House Delta Power")

        # (8) House consumption
        self.used_house_energy = Gauge("used_house_energy", "Used House Energy")
        self.used_house_delta_energy = Gauge("used_house_delta_energy", "Used House Delta Energy")
        self.used_house_power = Gauge("used_house_power", "Used House Power")

        # other
        self.imput_power = Gauge("input_power", "Input Power")
        self.active_power = Gauge("active_power", "Active Power")
        self.reactive_power = Gauge("reactive_power", "Reactive Power")

    def run_metrics_loop(self):
        """Metrics fetching loop"""

        print("entered run_metrics_loop()")

        while not self.is_shutdown:
            self.fetch()

            # show values for the last day on console:
            current_day = datetime.datetime.now().date()
            if current_day > self.current_day:
                temp = self.pv_facility.dump_last_day_values()
                print(temp)
                self.current_day = current_day
                
            delay_tick = 0.0
            while not self.is_shutdown and delay_tick < self.polling_interval_seconds:
                time.sleep(1.0)
                delay_tick += 1.0

        print("run_metrics_loop() finished.")

    def fetch(self):
        """
        Get metrics from application and refresh Prometheus metrics with
        new values.
        """

        # (1) PV exported to grid (grid export)
        grid_exported_kW = self.pv_facility.grid_exported_energy_1.get_last_value()
        self.grid_exported_energy.set(grid_exported_kW)
        grid_exported_delta_kW = self.pv_facility.grid_exported_energy_1.get_last_delta_value()
        self.grid_exported_delta_energy.set(grid_exported_delta_kW)
        grid_exported_delta_kWh = self.pv_facility.grid_exported_energy_1.get_last_delta_value_multipied_with_delta_time()
        self.grid_exported_delta_power.set(grid_exported_delta_kWh)

        # (2) Grid consumed from house (grid import)
        accumulated_grid_kW = self.pv_facility.grid_accumulated_energy_2.get_last_value()
        self.grid_accumulated_energy.set(accumulated_grid_kW)
        delta_grid_kW = self.pv_facility.grid_accumulated_energy_2.get_last_delta_value()
        self.grid_delta_energy.set(delta_grid_kW)
        delta_grid_kWh = self.pv_facility.grid_accumulated_energy_2.get_last_delta_value_multipied_with_delta_time()
        self.grid_delta_power.set(delta_grid_kWh)

        # (3) PV Yield (total = PV to grid + PV to house + Accu balance)
        yield_kW = self.pv_facility.accumulated_yield_energy_3.get_last_value()
        self.yield_energy.set(yield_kW)
        yield_delta_kW = self.pv_facility.accumulated_yield_energy_3.get_last_delta_value()
        self.yield_delta_energy.set(yield_delta_kW)
        yield_delta_kWh = self.pv_facility.accumulated_yield_energy_3.get_last_delta_value_multipied_with_delta_time()
        self.yield_delta_power.set(yield_delta_kWh)

        # (4) PV to storage
        storage_charge_kW = self.pv_facility.storage_total_charge_energy_4.get_last_value()
        self.storage_charge_energy.set(storage_charge_kW)
        storage_charge_delta_kW = self.pv_facility.storage_total_charge_energy_4.get_last_delta_value()
        self.storage_charge_delta_energy.set(storage_charge_delta_kW)
        storage_charge_delta_kWh = self.pv_facility.storage_total_charge_energy_4.get_last_delta_value_multipied_with_delta_time()
        self.storage_charge_delta_power.set(storage_charge_delta_kWh)

        # (5) Storage to house
        storage_discharge_kW = self.pv_facility.storage_total_discharge_energy_5.get_last_value()
        self.storage_discharge_energy.set(storage_discharge_kW)
        storage_discharge_delta_kW = self.pv_facility.storage_total_discharge_energy_5.get_last_delta_value()
        self.storage_discharge_delta_energy.set(storage_discharge_delta_kW)
        storage_discharge_delta_kWh = self.pv_facility.storage_total_discharge_energy_5.get_last_delta_value_multipied_with_delta_time()
        self.storage_discharge_delta_power.set(storage_discharge_delta_kWh)

        # (6) Storage balance
        storage_balance_delta_kw = storage_charge_delta_kW - storage_discharge_delta_kW
        self.storage_balance_delta_energy.set(storage_balance_delta_kw)
        storage_balance_delta_power_kwh = storage_charge_delta_kWh - storage_discharge_delta_kWh
        self.storage_balance_delta_power.set(storage_balance_delta_power_kwh)

        # (7) PV to house == (3) - (1)
        # PV Yield (3) - PV Export (1) // no: - Accu Balance (6))
        pv_consumed_house_energy_kw = yield_kW - grid_exported_kW - storage_balance_delta_kw
        self.pv_consumed_house_energy.set(pv_consumed_house_energy_kw)
        pv_consumed_house_delta_energy_kw = yield_delta_kW - grid_exported_delta_kW - storage_balance_delta_kw
        self.pv_consumed_house_delta_energy.set(pv_consumed_house_delta_energy_kw)
        pv_consumed_house_delta_power_kwh = yield_delta_kWh - grid_exported_delta_kWh # - storage_balance_delta_power_kwh     # TODO -> check -> accu discharge is contained in yield !
        self.pv_consumed_house_delta_power.set(pv_consumed_house_delta_power_kwh)

        # (8) House total consumption = Grid consumed from house (2) + PV Consumed from house (3) - PV Export (1)
        used_energy = accumulated_grid_kW + yield_kW - grid_exported_kW
        self.used_house_energy.set(used_energy)
        used_delta_energy = delta_grid_kW + yield_delta_kW - grid_exported_delta_kW
        self.used_house_delta_energy.set(used_delta_energy)
        used_power = delta_grid_kWh + yield_delta_kWh - grid_exported_delta_kWh
        self.used_house_power.set(used_power)

        # other data
        input_power = self.pv_facility.input_power_8.get_last_value()
        self.imput_power.set(input_power)
        active_power = self.pv_facility.active_power_9.get_last_value()
        self.active_power.set(active_power)
        reactive_power = self.pv_facility.reactive_power_10.get_last_value()
        self.reactive_power.set(reactive_power)

def main():
    """Main entry point"""

    polling_interval_seconds = int(os.getenv("POLLING_INTERVAL_SECONDS", "30"))
    exporter_port = int(os.getenv("EXPORTER_PORT", "9120"))
    print("Prometheus exporter for Huawei PV-Facility on port="+str(exporter_port))
    print("starting...")

    app_metrics = AppMetrics(
        polling_interval_seconds=polling_interval_seconds,
        is_debugging=False
    )

    def signal_handler(sig, frame):
        try:
            print('Ctrl+C pressed !') #,sig, frame)
            app_metrics.pv_facility.shutdown()
            time.sleep(2.0) # give time to stop the other threads
        except Exception as exc:
            print("EXCEPTION in signal handler:", exc)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    start_http_server(exporter_port)
    app_metrics.run_metrics_loop()

    # test this exporter in web browser: http://localhost:9120/

if __name__ == "__main__":
    main()
