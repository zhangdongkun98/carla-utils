import carla

from .agent_naive import NaiveAgent
from .agent_base_pseudo import BaseAgentPseudo


class NaiveAgentPseudo(BaseAgentPseudo, NaiveAgent):
    def __init__(self, config, client, world, town_map, vehicle, global_path=None, fast=False):
        NaiveAgent.__init__(self, config, client, world, town_map, vehicle, global_path)
        BaseAgentPseudo.__init__(self, config, client, world, town_map, vehicle, global_path, fast)
