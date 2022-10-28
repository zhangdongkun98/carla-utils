
from carla_utils.basic import Data
from carla_utils.agents import AgentListMaster


class RewardFunction(object):
    def __init__(self, config):
        self.config = config

    def run_step(self, state, action, agents_master: AgentListMaster, epoch_info: Data):
        return 0.0

