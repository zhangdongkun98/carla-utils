import carla

import multiprocessing as mp
import threading
import time

from ..system import Clock
from .agent_base import BaseAgent


class BaseAgentParallel(BaseAgent):
    def __init__(self, config, world, town_map, vehicle, sensors_master, global_path):
        super(BaseAgentParallel, self).__init__(config, world, town_map, vehicle, sensors_master, global_path)

        self.target = None
        self.clock_decision = Clock(self.decision_frequency)

        self.thread_stoped = False
        self.thread_control = threading.Thread(target=self._run_control, args=(lambda: self.thread_stoped,))
        self.thread_control.start()
        return

    def destroy(self):
        self.thread_stoped = True
        self.thread_control.join()
        return super().destroy()


    def _run_step_real(self, reference):
        self.clock_decision.tick_begin()
        self.target = self.get_target(reference)
        self.clock_decision.tick_end()
        return

    def _run_control(self, stop):
        while self.target == None:
            time.sleep(0.01)
        while True:
            self.clock.tick_begin()
            if self.goal_reached(5.0): self.extend_route()
            control = self.get_control(self.target)
            self.forward(control)
            if stop(): break
            self.clock.tick_end()
        self.vehicle_model(self.vehicle, carla.VehicleControl(brake=0.8))
        return
    
