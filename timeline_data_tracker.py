"""
    Simple Data Tracker to record timeline data in different resolutions
    (i. e. every 10 minutes and daily)
"""

import time
import datetime
import threading
import os.path

from file_utils import *

g_value = 0.0

def test_inc():
    global g_value
    g_value += 1.0
    value = g_value  # TODO -> implement
    return value

class SingleTimelineGrowingDataTrackerWithDeltaValues:
    """
        Measures a signal timeline with a given resolution/interval
        and saves the delta values for the days.
        This class is meant to be used for continues growing values,
        like operating hours counters.
    """

    MAX_ELEMENTS = 512

    EXTENSION = '.timeline'

    def __init__(self, signal_name, time_interval_in_sec = 60.0):
        self.signal_name = signal_name
        self.time_interval_in_sec = time_interval_in_sec
        self.signal_value_and_delta_cache = []  # items: (timestamp, integrated_value, delta_value)
        self.delta_day_cache = []               # items: (date, integrated_value, delta_value)
        self.last_measurement_date = datetime.datetime.now().date()
        self.last_delta_measurement_value = None
        self.last_day_measurement = None
        self._lock = threading.RLock()

        self.load_data()
# TODO -> write to database
        
        #print(self.delta_day_cache)

    def shutdown(self):
        self.save_data()

    def _get_persistence_full_file_name(self):
        return add_sdcard_path_if_available(g_sPersistencePath+os.sep+self.get_signal_name()+SingleTimelineGrowingDataTrackerWithDeltaValues.EXTENSION)

    def _set_persistence_data(self, data):
        if data is not None:
            self.signal_name = data[0]
            self.time_interval_in_sec = data[1]
            self.signal_value_and_delta_cache = data[2]
            self.delta_day_cache = data[3]
            self.last_measurement_date = data[4]
            self.last_delta_measurement_value = data[5]
            self.last_day_measurement = data[6]

    def _get_persistence_data(self):
        return (self.signal_name,
                self.time_interval_in_sec,
                self.signal_value_and_delta_cache,
                self.delta_day_cache,
                self.last_measurement_date,
                self.last_delta_measurement_value,
                self.last_day_measurement)

    def load_data(self):
        file_name = self._get_persistence_full_file_name()
        if os.path.exists(file_name):
            ok,data = read_data(file_name)
            if ok:
                self._set_persistence_data(data)

    def save_data(self):
        file_name = self._get_persistence_full_file_name()
        data = self._get_persistence_data()
        write_data(file_name, data)

    def get_signal_name(self):
        return self.signal_name

    # integrated or total value
    def get_last_values(self, index_from_last = -1):
        with self._lock:
            if len(self.signal_value_and_delta_cache) == 0 or len(self.signal_value_and_delta_cache) < abs(index_from_last):
                return (None, None, None)
            return self.signal_value_and_delta_cache[index_from_last]

    # integrated or total value
    def get_last_value(self, index_from_last = -1):
        return self.get_last_values(index_from_last)[1]

    def get_last_delta_value(self, index_from_last = -1):
        return self.get_last_values(index_from_last)[2]

    # kWh * t -> kW
    def get_last_delta_value_multipied_with_delta_time(self, index_from_last = -1):
        _temp = self.get_last_values(index_from_last)
        _prev_temp = self.get_last_values(index_from_last-1)
        if _temp[2] is not None and _prev_temp[2] is not None:
            return float(_temp[2]) / float((_temp[0]-_prev_temp[0]).total_seconds()) * 60.0 * 60.0  # sec -> min -> hour
        return 0.0

    def _get_last_day_value(self, index_from_last = -1):
        with self._lock:
            if len(self.delta_day_cache) == 0:
                return (None, None, None)
            return self.delta_day_cache[index_from_last]

    def get_last_day_value(self, index_from_last = -1):
        return self._get_last_day_value(index_from_last)[1]

    def get_last_day_delta_value(self, index_from_last = -1):
        return self._get_last_day_value(index_from_last)[2]

    def do_measurement(self, fcnMeasure):
        with self._lock:
            value = fcnMeasure()
            timestamp = self.add_measurement(value)
            return (timestamp, value)

    def add_measurement(self, current_value):
        with self._lock:
            current_timestamp = datetime.datetime.now()
            current_date = current_timestamp.date()

            # handle delta values
            if self.last_delta_measurement_value is not None and current_value is not None:
                delta_value = current_value - self.last_delta_measurement_value
            else:
                delta_value = 0.0 # == initial value
            # update last measurement (this is the delta reference)
            self.last_delta_measurement_value = current_value
            self._append_value_and_delta_value(current_timestamp, current_value, delta_value)

            # check for caching of delta day values
            if current_date > self.last_measurement_date:
                if self.last_day_measurement is not None and current_value is not None:
                    delta_for_last_day = current_value - self.last_day_measurement
                else:
                    delta_for_last_day = current_value
                self.last_day_measurement = current_value
                self._append_delta_day_value(self.last_measurement_date, current_value, delta_for_last_day)
            self.last_measurement_date = current_date

            return current_timestamp

    def _resize_list_if_needed(self, lst):
        if len(lst) > SingleTimelineGrowingDataTrackerWithDeltaValues.MAX_ELEMENTS:
            lst = lst[len(lst)-SingleTimelineGrowingDataTrackerWithDeltaValues.MAX_ELEMENTS:]
        return lst

    def _append_value_and_delta_value(self, timestamp, value, delta_value):
        with self._lock:
            self.signal_value_and_delta_cache.append( (timestamp, value, delta_value) )
            # resize array if to large !
            self.signal_value_and_delta_cache = self._resize_list_if_needed(self.signal_value_and_delta_cache)

            # TODO -> write to database ?

    def _append_delta_day_value(self, day, value, delta_value):
        with self._lock:
            self.delta_day_cache.append( (day, value, delta_value) )
            # resize array if to large !
            self.delta_day_cache = self._resize_list_if_needed(self.delta_day_cache)

            # TODO -> write to database ?

    def run(self):
        while True:
            measurement = self.do_measurement(test_inc)
            print(measurement)
            print(self.signal_value_and_delta_cache)
            print(self.delta_day_cache)
            time.sleep(self.time_interval_in_sec)

    def loop(self):
        while True:
            print(".", end=None, flush=True)
            time.sleep(1)

if __name__ == '__main__':
    test = SingleTimelineGrowingDataTrackerWithDeltaValues("test", 5)
    print(test)
    loop_thread = threading.Thread(target=test.loop, args=())
    loop_thread.start()
    measure_thread = threading.Thread(target=test.run, args=())
    measure_thread.start()
    print("looping...")
    #test.run()
    #test.loop()
