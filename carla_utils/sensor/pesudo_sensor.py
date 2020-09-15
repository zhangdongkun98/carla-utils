

class Sensor(object):
    def __init__(self, transform, config):
        self.type_id = 'sensor.camera.rgb'
        self.transform = transform
        self.attributes = dict()
        self.attributes['role_name'] = config['role_name']
        self.attributes['image_size_x'] = str( config['img_length'] )
        self.attributes['image_size_y'] = str( config['img_width'] )
        self.attributes['fov'] = str( config['fov'] )