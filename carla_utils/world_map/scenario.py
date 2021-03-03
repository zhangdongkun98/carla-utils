



class Scenario(object):
    default_maps = ['Town01', 'Town02', 'Town03', 'Town04', 'Town05', 'Town06', 'Town07', 'Town10HD']
    def __init__(self, town_map):
        self.town_map = town_map
        self.map_name = town_map.name
        self.spawn_points = self._get_spawn_points()
        return
    
    def _get_spawn_points(self):
        return [t.location for t in self.town_map.get_spawn_points()]

