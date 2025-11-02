"""
    Read information from the PV facility.
"""

import sys
import asyncio
import threading
import time
import signal
import traceback

import timeline_data_tracker as tldt

from file_utils import *

from huawei_solar import AsyncHuaweiSolar, ConnectionException, register_names as rn
#from huawei_solar import HuaweiSolarBridge

import pv_facility_sqldb as sqldb

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
    client = await AsyncHuaweiSolar.create("lwip", 502, slave_id)  # ip=192.168.178.49  # TODO: timeout=30.0, cooldown_time=10.0 ???
    #print("-->",client)

    # Tägliche Werte
    # - tägliche Einspeisung ins Netz             ok -> Delta (1)
    # - täglicher Bezug vom Netz                  ok -> Delta (2)
    # - tägliche Erzeugung PV-Anlage              ok -> Delta (3)      --> ist PV Verbrauch für Haus -> PV gesamt = PV Haus + Akku Load + Grid export
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
    #                                            (11)
    results6 = await client.get_multiple([rn.INTERNAL_TEMPERATURE], slave_id)
    #                                            (12)
    results7 = await client.get_multiple([rn.STORAGE_UNIT_1_BATTERY_TEMPERATURE], slave_id)

#TODO    # ACCUMULATED_YIELD_ENERGY == Daily Energy? == Total Consumption

    # close connection to give other client the possibility to query data
    #await client.stop()

    return results1 + results2 + results3 + results4 + results5 + results6 + results7

def _check_for_float(value, default_val=0.0):
    if value is not None:
        return float(value)
    return default_val

def _append_to_sql_db(data_str):
    try:
        conn = sqldb.get_db_connection()
        cursor = conn.cursor()
        sqldb.insert_data_string(cursor, data_str)
        conn.commit()
        conn.close()
    except Exception as exc:
        print("EXCEPTION in _append_to_sql_db():", exc)

class PV_Facility:

    DELAY = 300.0   # seconds == 5 minutes --> ca. 288 measurements per day

    AUTO_SAVE_SECONDS = 3600.0 # == 60.0 * 60.0

    def __init__(self, is_debugging=False, fcnTriggerShutdown=None):
        self.is_debugging = is_debugging
        self.is_shutdown = False
        self.fcn_trigger_shutdown = fcnTriggerShutdown
        self.last_save_timestamp = None
        self.current_day = datetime.datetime.now().date()
        self.grid_exported_energy_1 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.GRID_EXPORTED_ENERGY)
        self.grid_accumulated_energy_2 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.GRID_ACCUMULATED_ENERGY)
        self.accumulated_yield_energy_3 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.ACCUMULATED_YIELD_ENERGY)
        self.storage_total_charge_energy_4 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.STORAGE_UNIT_1_TOTAL_CHARGE)
        self.storage_total_discharge_energy_5 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.STORAGE_UNIT_1_TOTAL_DISCHARGE)
        self.input_power_8 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.INPUT_POWER)
        self.active_power_9 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.ACTIVE_POWER)
        self.reactive_power_10 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.REACTIVE_POWER)
        self.internal_temperature_11 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.INTERNAL_TEMPERATURE)
        self.storage_unit_1_battery_temperature_12 = tldt.SingleTimelineGrowingDataTrackerWithDeltaValues(rn.STORAGE_UNIT_1_BATTERY_TEMPERATURE)

    def shutdown(self):
        self.grid_exported_energy_1.shutdown()
        self.grid_accumulated_energy_2.shutdown()
        self.accumulated_yield_energy_3.shutdown()
        self.storage_total_charge_energy_4.shutdown()
        self.storage_total_discharge_energy_5.shutdown()
        self.input_power_8.shutdown()
        self.active_power_9.shutdown()
        self.reactive_power_10.shutdown()
        self.internal_temperature_11.shutdown()
        self.storage_unit_1_battery_temperature_12.shutdown()
        self.is_shutdown = True

    def save_data(self):
        self.grid_exported_energy_1.save_data()
        self.grid_accumulated_energy_2.save_data()
        self.accumulated_yield_energy_3.save_data()
        self.storage_total_charge_energy_4.save_data()
        self.storage_total_discharge_energy_5.save_data()
        self.input_power_8.save_data()
        self.active_power_9.save_data()
        self.reactive_power_10.save_data()
        self.internal_temperature_11.save_data()
        self.storage_unit_1_battery_temperature_12.save_data()
        self.last_save_timestamp = datetime.datetime.now()

    def save_if_needed(self):
        if self.last_save_timestamp is None:
            self.last_save_timestamp = datetime.datetime.now()
        if (datetime.datetime.now() - self.last_save_timestamp).seconds > PV_Facility.AUTO_SAVE_SECONDS:
            self.save_data()

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

    # - tägliche Einspeisung ins Netz             ok -> Delta (1)  ==> Ins Netz eingespeist
    # - täglicher Bezug vom Netz                  ok -> Delta (2)  ==> Vom Netz
    # - tägliche Erzeugung PV-Anlage              ok -> Delta (3) 
    # - tägliches Akku laden                      ok -> Delta (4)
    # - tägliches Akku entladen                   ok -> Delta (5)
    # - tägliche Bilanz Akku (netto aufgeladen oder entladen)      ==> Akku Bilanz       (6) = (4) - (5)
    # - täglicher Verbrauch von PV-Anlage                          ==> Von PV (Anlage)   (7) = (3) - (1)
    # - täglicher Verbrauch des Hauses                             ==> Verbrauch (Haus)  (8) = (3) - (1) + (2)
    #
    # Solar Fusion Output Diagram:
    #
    #   (3) + (6)                          (8)
    #       ^                               ^
    #       |                               |
    #       |---------------------          |---------------------
    #   (3) - (1) + (6)         (1)        (7) = (3) - (1)      (2)

    def dump_last_day_values(self, index_from_last = -1):
        """
            Example (old code):

            Values for last day (2024-02-21)
            --------------------------------
            PV Export to Grid:      2.11 kWh (1)
            Consumed from Grid:     1.30 kWh (2)
            PV Yield:              14.33 kWh (3)  -> 14.33 - 1.42 = 12.91 bzw. 15.   (Produktion) = (3) + (6) ???
            Accu Balance:           1.42 kWh (6)  // war noch geladen vom Vortag und hat zum Ende des Tages noch Restladung (ca. 1.19 kWh)  -> WARUM HIER KEINE Akku Bilanz 0 ????
            PV consumed housed:    10.80 kWh (7)  --> besser: (3) + (6) => 14.33 + 1.42 = 15.75
            House Consumed:        12.10 kWh (8)  --> besser: (2) + (7) => 1.3 + 15.75 = 17.05      gesucht 15.3                                                                   13.39 <--> 13.3  -> Verbrucht noch Akku vom Vortag und gibt Akku an folgenden Tag
                                                              (2) + (3) => 1.3 + 14.33 = 15.62 == 15.3

            Values for last day (2024-02-22)
            --------------------------------
            PV Export to Grid:      0.00 kWh (1)
            Consumed from Grid:    10.27 kWh (2)
            PV Yield:               6.38 kWh (3)  -> 6.38 + 1.19 = 7.57 bzw. 5.19
            Accu Balance:          -1.19 kWh (6)  // war noch geladen vom Vortag
            PV consumed housed:     7.57 kWh (7)  -> wäre 6.38 = 7.57 - 1.19 => 6.38 (3) - 1.19 (6) = 5.19 (ok) -> Verbraucht => (3) + (6)
            House Consumed:        17.84 kWh (8)  -> wird korrigiert durch neuen Wert von (7) -> ohne akku balance -> 16.65 (ok) => (2) + (7) = 10.27 + 5.19 = 15.46 == 16.63      5.17 <--> 6.38  -> PV hat 5.17 produziert und noch 1.19 vom Akku wegen Vortag ans Haus abgeteben
                                                            (2) + (3) => 10.27 + 6.38 = 16.65 == 16.63

    Nach SourceCode Change:
                                                                                                                                                                                                                                                                                                                                                                    VerbrauchT   Von PV
            Values for last day (2024-02-23), Timestamp: 2024-02-25 00:00:05.130626
            --------------------------------
            PV Export to Grid:      1.02 kWh
            Consumed from Grid:     5.47 kWh
            PV Yield:              11.40 kWh   -> 11.4 + 5.47 = 16.87
            Accu Balance:          -0.07 kWh
            PV consumed house:     11.33 kWh   
            House Consumed:        16.87 kWh
        """
        s = ''
        date_for_dump_data = datetime.datetime.now().date()-datetime.timedelta(days=abs(index_from_last))
        s += f'Values for last day ({date_for_dump_data}), Timestamp: {datetime.datetime.now()}\n'
        s +=  '--------------------------------\n'
        grid_export_kwh = self.grid_exported_energy_1.get_last_day_delta_value(index_from_last)
        grid_import_kwh = self.grid_accumulated_energy_2.get_last_day_delta_value(index_from_last)
        pv_yield_kwh = self.accumulated_yield_energy_3.get_last_day_delta_value(index_from_last)
        accu_charge_kwh = self.storage_total_charge_energy_4.get_last_day_delta_value(index_from_last)
        total_accu_charge_kwh = self.storage_total_charge_energy_4.get_last_day_value(index_from_last)
        accu_discharge_kwh = self.storage_total_discharge_energy_5.get_last_day_delta_value(index_from_last)
        total_accu_discharge_kwh = self.storage_total_discharge_energy_5.get_last_day_value(index_from_last)
        accu_balance_kwh = accu_charge_kwh - accu_discharge_kwh if accu_charge_kwh is not None and accu_discharge_kwh is not None else 0.0
        s += f'PV Export to Grid:  {grid_export_kwh:8.2f} kWh\n'       # (1)    # == Huawei: Ins Netz eingespeist
        s += f'Consumed from Grid: {grid_import_kwh:8.2f} kWh\n'       # (2)    # == Huawei: Vom Netz
        s += f'PV Yield:           {pv_yield_kwh:8.2f} kWh\n'          # (3)
        s += f'Accu Balance:       {accu_balance_kwh:8.2f} kWh (total_charge={_check_for_float(total_accu_charge_kwh):8.2f} kWh, total_discharge={_check_for_float(total_accu_discharge_kwh):8.2f})\n'      # (6)
        pv_consumed_house_kwh = pv_yield_kwh - grid_export_kwh
        s += f'PV consumed house:  {pv_consumed_house_kwh:8.2f} kWh\n' # (7)    # == Huawei: Von PV ==>    (7) = (3) - (1)
        house_consumed_kwh = grid_import_kwh + pv_yield_kwh - grid_export_kwh
        s += f'House Consumed:     {house_consumed_kwh:8.2f} kWh\n'    # (8)    # == Huawei: Verbrauch ==> (8) = (3) - (1) + (2)
        # For sql command like:
        # INSERT INTO `DailyPVEnergy` (`Timestamp`, `EnergyExportGrid`, `EnergyImportGrid`, `EnergyYieldPV`, `EnergyAccuCharge`, `EnergyAccuDischarge`, `EnergyPVToHouse`, `EnergyIntoHouse`) VALUES
        # ('2024-02-29 19:23:07', 26.1, 4.2, 27, 0, 0, 12.3, 14.52);
# TODO: Zeit auf 23:59:59 fuer vorherigen Tag setzen!        
        sql_insert_data = f"('{date_for_dump_data}',{grid_export_kwh:1.3f},{grid_import_kwh:1.3f},{pv_yield_kwh:1.3f},{_check_for_float(total_accu_charge_kwh):1.3f},{_check_for_float(total_accu_discharge_kwh):1.3f},{pv_consumed_house_kwh:1.3f},{house_consumed_kwh:1.3f})"
        s += sql_insert_data
        # return string which can be used to insert values into SQL database
        return (s, sql_insert_data)
    
    def dump_all_day_values(self, no_of_days=None):
        s = ''
        max_count = len(self.grid_exported_energy_1.delta_day_cache)
        if no_of_days is None:
            max_output = max_count
        else:
            max_output = no_of_days
        for i in range(max_output):
            _temp, _sql_insert_data = self.dump_last_day_values(-(max_output-i))
            s += _temp
            s += '\n'
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
        self.internal_temperature_11.do_measurement(lambda: pv_infos[8].value)
        self.storage_unit_1_battery_temperature_12.do_measurement(lambda: pv_infos[9].value)

    def _test_for_restart_or_sleep(self, measure_error_counter, sActFileName):
        measure_error_counter += 1
        if measure_error_counter > 30:  # try to restart every 30 minutes !
            append_to_file(sActFileName, f"WARNING: shutdown pv_facility_reader.py now... (timestamp={datetime.datetime.now()})", sPrefix="#")
            self.shutdown()     # saves also the data !
            if self.fcn_trigger_shutdown is not None:
                self.fcn_trigger_shutdown()
        else:
            time.sleep(60)
        return measure_error_counter

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
                       self.reactive_power_10.get_signal_name()+POSTFIX,
                       self.internal_temperature_11.get_signal_name(),
                       self.internal_temperature_11.get_signal_name()+POSTFIX,
                       self.storage_unit_1_battery_temperature_12.get_signal_name(),
                       self.storage_unit_1_battery_temperature_12.get_signal_name()+POSTFIX
                       ]
        append_to_file(sActFileName,aDataHeader,sPrefix="#")

        # dump the infos of the last days at startup !
        print(self.dump_all_day_values(14))

        measure_error_counter = 0
        connected_with_huawei_pv_converter = False
        while not self.is_shutdown:
            try:
                # show values for the last day on console:
                _now = datetime.datetime.now()
                current_day = _now.date()
                if current_day > self.current_day:
                    value_txt, _sql_insert_data = self.dump_last_day_values() #index_from_last=0)  # 0 because the values of the new day was not written into the timeline_date_tracker !
                    print(value_txt)
                    # append daily PV values into text files
                    append_to_file(add_sdcard_path_if_available(g_sDailyPvValuesFile),_sql_insert_data)
                    append_to_file(sActFileName,_sql_insert_data,sPrefix="#",sPostfix=f" # timestamp: {_now}")
                    # insert daily PV values into MariaDB
# TODO -> trage um Mitternacht auf jeden Fall die aktuellen Werte fuer den letzten Tag in die Datenbank ein ! ??? -> self.current_day persistent machen???
                    _append_to_sql_db(_sql_insert_data)
                    self.current_day = current_day

                self._measure()
                measure_error_counter = 0

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
                vals_9 = self.internal_temperature_11.get_last_values()
                vals_10 = self.storage_unit_1_battery_temperature_12.get_last_values()
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
                aLineInfo.append(vals_9[1])
                aLineInfo.append(vals_9[2])
                aLineInfo.append(vals_10[2])
                sActFileName,aActDate,bNewFile = map_date_to_filename(sFileName,self.is_debugging,aActDate)
                if bNewFile:
                    append_to_file(sActFileName,aDataHeader,sPrefix="#")
                append_to_file(sActFileName,aLineInfo,sPostfix=f" # ts={datetime.datetime.now()}")

                # save the data every hour
                self.save_if_needed()

                connected_with_huawei_pv_converter = True

                delay_tick = 0.0
                while not self.is_shutdown and delay_tick < PV_Facility.DELAY:
                    time.sleep(1.0)
                    delay_tick += 1.0

            except ConnectionException as ex:
                # Huawei PV converter may be switched off !
                s = f"EXCEPTION in pv_run() loop at timestamp {datetime.datetime.now()}: Could not connect to Huawei PV Converter -> switched off or power save modus? measure_error_counter={measure_error_counter}"
                print(s)
                traceback.print_exception(ex)
                if connected_with_huawei_pv_converter:
                    # write this message only once into data file if the connection to the huawei pv converter is lost because of power save modus
                    append_to_file(sActFileName, s, sPrefix="#")
                connected_with_huawei_pv_converter = False
                measure_error_counter = self._test_for_restart_or_sleep(measure_error_counter, sActFileName)
            except Exception as ex:
                s = f"EXCEPTION in pv_run() loop at timestamp {datetime.datetime.now()}:\n{type(ex)}\n{ex}"
                print(s)
                traceback.print_exception(ex)
                #traceback.print_stack()
                s += ' -> '
                s += ''.join(traceback.format_exception(ex))
                s = s.replace('\n',' ')
                append_to_file(sActFileName, s, sPrefix="#")
                measure_error_counter = self._test_for_restart_or_sleep(measure_error_counter, sActFileName)

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
