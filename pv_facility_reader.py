"""
    Read information from the PV facility.
"""

import sys
import asyncio
import threading
import time
import signal

import timeline_data_tracker as tldt

from file_utils import *

from huawei_solar import AsyncHuaweiSolar, register_names as rn
#from huawei_solar import HuaweiSolarBridge

# async def read_pv_facility_grid_infos(client, slave_id):
#     results = await client.get_multiple([rn.GRID_EXPORTED_ENERGY, rn.GRID_ACCUMULATED_ENERGY], slave_id)
#     return results

# async def read_pv_facility_yield_energy_infos(client, slave_id):
#     results = await client.get_multiple([rn.ACCUMULATED_YIELD_ENERGY, rn.DAILY_YIELD_ENERGY], slave_id)
#     return results

# async def read_pv_facility_storage_infos(client, slave_id):
#     results = await client.get_multiple([rn.STORAGE_UNIT_1_TOTAL_CHARGE, rn.STORAGE_UNIT_1_TOTAL_DISCHARGE], slave_id)
#     return results

# async def read_pv_facility_infos_new():
#     slave_id = 1
#     client = await AsyncHuaweiSolar.create("lwip", 502, slave_id)  # ip=192.168.178.49
#     print("-->",client)

# #    result = asyncio.gather(read_pv_facility_grid_infos(client, slave_id), read_pv_facility_yield_energy_infos(client, slave_id), read_pv_facility_storage_infos(client, slave_id))
#     result = asyncio.gather(read_pv_facility_grid_infos(client, slave_id))
#     print(result)
#     return result

async def read_pv_facility_infos():
    slave_id = 1
    client = await AsyncHuaweiSolar.create("lwip", 502, slave_id)  # ip=192.168.178.49
    #print("-->",client)

    # Tägliche Werte
    # - tägliche Einspeisung ins Netz             ok -> Delta (1)
    # - täglicher Bezug vom Netz                  ok -> Delta (2)
    # - tägliche Erzeugung PV-Anlage              ok -> Delta (3)      --> ist PV Verbrauch für Haus -> PV gesamt = PV Haus + Akku Load (+ Grid export) ???
    # - tägliches Akku laden                      ok -> Delta (4)
    # - tägliches Akku entladen                   ok -> Delta (5)
    # - tägliche Bilanz Akku (netto aufgeladen oder entladen)  Delta Akku geladen (4) - Delta Akku entladen (5) = Akku Bilanz (6)
    # - täglicher Verbrauch von PV-Anlage         ?  => Erzeugung (3) - Enspeisung (1) - Akku Bilanz (6) = Verbrauch von PV-Anlage (7)
    # - täglicher Verbrauch des Hauses            ?  => Bezug vom Netz (2) + Verbrauch von PV-Anlage (7) = Verbrauch Haus (8)

    #                                                 (1)                        (2)
    results1 = await client.get_multiple([rn.GRID_EXPORTED_ENERGY, rn.GRID_ACCUMULATED_ENERGY], slave_id)
    #                                                 (3)
    results2 = await client.get_multiple([rn.ACCUMULATED_YIELD_ENERGY], slave_id)  # maybe later: , rn.DAILY_YIELD_ENERGY
    #                                                 (4)                        (5)
    results3 = await client.get_multiple([rn.STORAGE_UNIT_1_TOTAL_CHARGE, rn.STORAGE_UNIT_1_TOTAL_DISCHARGE], slave_id)
    #                                            (8)
    results4 = await client.get_multiple([rn.INPUT_POWER], slave_id)
    #                                            (9)               (10)
    results5 = await client.get_multiple([rn.ACTIVE_POWER, rn.REACTIVE_POWER], slave_id)

#TODO    # ACCUMULATED_YIELD_ENERGY == Daily Energy? == Total Consumption

    # close connection to give other client the possibility to query data
    #await client.stop()

    return results1 + results2 + results3 + results4 + results5

class PV_Facility:

    DELAY = 300.0   # seconds == 5 minutes --> ca. 288 measurements per day

    def __init__(self, is_debugging=False):
        self.is_debugging = is_debugging
        self.is_shutdown = False
        self.grid_exported_energy_1 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.GRID_EXPORTED_ENERGY)
        self.grid_accumulated_energy_2 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.GRID_ACCUMULATED_ENERGY)
        self.accumulated_yield_energy_3 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.ACCUMULATED_YIELD_ENERGY)
        self.storage_total_charge_energy_4 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.STORAGE_UNIT_1_TOTAL_CHARGE)
        self.storage_total_discharge_energy_5 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.STORAGE_UNIT_1_TOTAL_DISCHARGE)
        self.input_power_8 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.INPUT_POWER)
        self.active_power_9 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.ACTIVE_POWER)
        self.reactive_power_10 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.REACTIVE_POWER)

    def shutdown(self):
        self.grid_exported_energy_1.shutdown()
        self.grid_accumulated_energy_2.shutdown()
        self.accumulated_yield_energy_3.shutdown()
        self.storage_total_charge_energy_4.shutdown()
        self.storage_total_discharge_energy_5.shutdown()
        self.input_power_8.shutdown()
        self.active_power_9.shutdown()
        self.reactive_power_10.shutdown()
        self.is_shutdown = True

    # (1) PV -> grid (== PV Export)
    def get_grid_exported_energy_day(self, index_from_last = -1):
        return self.grid_exported_energy_1.get_last_day_delta_value(index_from_last)

    # (2) House <- grid
    def get_grid_accumulated_energy_day(self, index_from_last = -1):
        return self.grid_accumulated_energy_2.get_last_day_delta_value(index_from_last)

    # (3) PV Yield -> (== PV to Grid + PV to House + PV to Akku Balance)
    def get_accumulated_yield_energy_day(self, index_from_last = -1):
        return self.accumulated_yield_energy_3.get_last_day_delta_value(index_from_last)

    # (4) Akku <-
    def get_storage_total_charge_day(self, index_from_last = -1):
        return self.storage_total_charge_energy_4.get_last_day_delta_value(index_from_last)

    # (5) Akku ->
    def get_storage_total_discharge_day(self, index_from_last = -1):
        return self.storage_total_discharge_energy_5.get_last_day_delta_value(index_from_last)
    
    # (6) Akku +/-
    def get_storage_balance_day(self, index_from_last = -1):
        return self.get_storage_total_charge_day(index_from_last) - self.get_storage_total_discharge_day(index_from_last)

    # (7) PV -> House (PV consumption house == PV Yield (3) - PV Export (1) - Accu Balance (6))
    def get_used_yield_enery_day(self, index_from_last = -1):
        return self.get_accumulated_yield_energy_day(index_from_last) - self.get_grid_exported_energy_day(index_from_last) - self.get_storage_balance_day(index_from_last)
    
    # (8) House <- (== Consumption house = PV Yield (3) - PV Export (1) - Accu Balance (6) + Grid consumption house (2) == PV consumption house (7) + Grid consumption house (2))
    def get_total_used_energy_day(self, index_from_last = -1):
        return self.get_grid_accumulated_energy_day(index_from_last) + self.get_used_yield_enery_day(index_from_last)

    # - tägliche Einspeisung ins Netz             ok -> Delta (1)
    # - täglicher Bezug vom Netz                  ok -> Delta (2)
    # - tägliche Erzeugung PV-Anlage              ok -> Delta (3)
    # - tägliches Akku laden                      ok -> Delta (4)
    # - tägliches Akku entladen                   ok -> Delta (5)
    # - tägliche Bilanz Akku (netto aufgeladen oder entladen)  Delta Akku geladen (4) - Delta Akku entladen (5) = Akku Bilanz (6)
    # - täglicher Verbrauch von PV-Anlage         ?  => PV Erzeugung (3) - Enspeisung (1) - Akku Bilanz (6) = Verbrauch von PV-Anlage (7)
    # - täglicher Verbrauch des Hauses            ?  => Bezug vom Netz (2) + Verbrauch von PV-Anlage (7) = Verbrauch Haus (8)

    def dump_last_day_values(self):
        s = ''
        s += f'Values for last day ({datetime.datetime.now().date()-datetime.timedelta(days=1)})\n'
        s +=  '--------------------------------\n'
        grid_export_kwh = self.grid_exported_energy_1.get_last_day_delta_value()
        grid_import_kwh = self.grid_accumulated_energy_2.get_last_day_delta_value()
        pv_yield_kwh = self.accumulated_yield_energy_3.get_last_day_delta_value()
        accu_balance_kwh = self.storage_total_charge_energy_4.get_last_day_delta_value() - self.storage_total_discharge_energy_5.get_last_day_delta_value()
        s += f'PV Export to Grid:  {grid_export_kwh:8.2f} kWh\n'       # (1)
        s += f'Consumed from Grid: {grid_import_kwh:8.2f} kWh\n'       # (2)
        s += f'PV Yield:           {pv_yield_kwh:8.2f} kWh\n'          # (3)
        s += f'Accu Balance:       {accu_balance_kwh:8.2f} kWh\n'      # (6)
        pv_consumed_house_kwh = pv_yield_kwh - grid_export_kwh - accu_balance_kwh
        s += f'PV consumed housed: {pv_consumed_house_kwh:8.2f} kWh\n' # (7)
        house_consumed_kwh = grid_import_kwh + pv_consumed_house_kwh
        s += f'House Consumed:     {house_consumed_kwh:8.2f} kWh\n'    # (8)
        return s

    def _measure(self):
        if self.is_debugging:
            print("##>> start reading...")
        pv_infos = asyncio.run(read_pv_facility_infos())
        if self.is_debugging:
            print("##>>", pv_infos)
        self.grid_exported_energy_1.do_measurement(lambda: pv_infos[0].value)
        self.grid_accumulated_energy_2.do_measurement(lambda: pv_infos[1].value)
        self.accumulated_yield_energy_3.do_measurement(lambda: pv_infos[2].value)
        self.storage_total_charge_energy_4.do_measurement(lambda: pv_infos[3].value)
        self.storage_total_discharge_energy_5.do_measurement(lambda: pv_infos[4].value)
        self.input_power_8.do_measurement(lambda: pv_infos[5].value)
        self.active_power_9.do_measurement(lambda: pv_infos[6].value)
        self.reactive_power_10.do_measurement(lambda: pv_infos[7].value)

    def pv_run(self):
        print("entered pv_run() thread")
        POSTFIX = "_delta"
        sFileName = g_sPvLogFile
        sActFileName,aActDate,bNewFile = map_date_to_filename(sFileName,self.is_debugging,None)
        aDataHeader = ['timestamp',
                       self.grid_exported_energy_1.get_signal_name(),
                       self.grid_exported_energy_1.get_signal_name()+POSTFIX,
                       self.grid_accumulated_energy_2.get_signal_name(),
                       self.grid_accumulated_energy_2.get_signal_name()+POSTFIX,
                       self.accumulated_yield_energy_3.get_signal_name(),
                       self.accumulated_yield_energy_3.get_signal_name()+POSTFIX,
                       self.storage_total_charge_energy_4.get_signal_name(),
                       self.storage_total_charge_energy_4.get_signal_name()+POSTFIX,
                       self.storage_total_discharge_energy_5.get_signal_name(),
                       self.storage_total_discharge_energy_5.get_signal_name()+POSTFIX,
                       self.input_power_8.get_signal_name(),
                       self.input_power_8.get_signal_name()+POSTFIX,
                       self.active_power_9.get_signal_name(),
                       self.active_power_9.get_signal_name()+POSTFIX,
                       self.reactive_power_10.get_signal_name(),
                       self.reactive_power_10.get_signal_name()+POSTFIX
                       ]
        append_to_file(sActFileName,aDataHeader,sPrefix="#")
        while not self.is_shutdown:
            self._measure()
            if self.is_debugging:
                print("GRID_EXPORTED:", self.grid_exported_energy_1.signal_value_and_delta_cache)
                print("GRID_CONSUMED:", self.grid_accumulated_energy_2.signal_value_and_delta_cache)
                print("YIELD:        ", self.accumulated_yield_energy_3.signal_value_and_delta_cache)
                print("CHARGED:      ", self.storage_total_charge_energy_4.signal_value_and_delta_cache)
                print("DISCHARGED:   ", self.storage_total_discharge_energy_5.signal_value_and_delta_cache)
            aLineInfo = []
            vals_1 = self.grid_exported_energy_1.get_last_values()
            vals_2 = self.grid_accumulated_energy_2.get_last_values()
            vals_3 = self.accumulated_yield_energy_3.get_last_values()
            vals_4 = self.storage_total_charge_energy_4.get_last_values()
            vals_5 = self.storage_total_discharge_energy_5.get_last_values()
            vals_6 = self.input_power_8.get_last_values()
            vals_7 = self.active_power_9.get_last_values()
            vals_8 = self.reactive_power_10.get_last_values()
            aLineInfo.append(vals_1[0]) # write timestamp of measurement
            aLineInfo.append(vals_1[1])
            aLineInfo.append(vals_1[2])
            aLineInfo.append(vals_2[1])
            aLineInfo.append(vals_2[2])
            aLineInfo.append(vals_3[1])
            aLineInfo.append(vals_3[2])
            aLineInfo.append(vals_4[1])
            aLineInfo.append(vals_4[2])
            aLineInfo.append(vals_5[1])
            aLineInfo.append(vals_5[2])
            aLineInfo.append(vals_6[1])
            aLineInfo.append(vals_6[2])
            aLineInfo.append(vals_7[1])
            aLineInfo.append(vals_7[2])
            aLineInfo.append(vals_8[1])
            aLineInfo.append(vals_8[2])
            sActFileName,aActDate,bNewFile = map_date_to_filename(sFileName,self.is_debugging,aActDate)
            if bNewFile:
                append_to_file(sActFileName,aDataHeader,sPrefix="#")
            append_to_file(sActFileName,aLineInfo)

            delay_tick = 0.0
            while not self.is_shutdown and delay_tick < PV_Facility.DELAY:
                time.sleep(1.0)
                delay_tick += 1.0

        print("pv_run() thread stoped.")

    def loop(self):
        while not self.is_shutdown and True:
            if self.is_debugging:
                print(".", self.grid_accumulated_energy_2.get_last_value(), "/", self.storage_total_discharge_energy_5.get_last_value(), "/", self.storage_total_discharge_energy_5.get_last_delta_value(), ".", end="", flush=True)
            time.sleep(1)

if __name__ == '__main__':
    obj = PV_Facility(is_debugging=True)
    print(obj)

    def signal_handler(sig, frame):
        try:
            print('Ctrl+C pressed !') #,sig, frame)
            obj.shutdown()
            print('called shutdown.')
        except Exception as exc:
            print("EXCEPTION in signal handler:", exc)
        print('exit(0) now...')
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    measure_thread = threading.Thread(target=obj.pv_run, args=())
    measure_thread.start()
    #obj.run()
    obj.loop()
