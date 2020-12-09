
import time


class Clock(object):
    def __init__(self, frequency):
        self.frequency = float(frequency)
        self.dt = 1 / self.frequency
        self._tick = None

    def tick_begin(self):
        self._tick = time.time()
    def tick_end(self):
        sleep_time = self.dt - time.time() + self._tick
        time.sleep( max(0, sleep_time) )
