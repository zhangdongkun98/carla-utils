import carla

from .agent_base import BaseAgent


class NaiveAgent(BaseAgent):
    def __init__(self, config, world, town_map, vehicle, global_path=None):
        super(NaiveAgent, self).__init__(config, world, town_map, vehicle, global_path)
    

    def run_step(self, client):
        """
        
        
        Args:
            client: carla.Client
        
        Returns:
            None
        """

        
        
