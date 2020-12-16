
import os
from os.path import join
import time


def create_dir(config, model_name):
    '''
        create dir and save config
        Args:
            config: need to contain:
                config.description
    '''
    dataset_name = model_name + '/' + str(int(time.time())) + '----' + config.description
    print('create dir: ', dataset_name)
    log_path = join('results', dataset_name, 'log')
    save_model_path = join('results', dataset_name, 'saved_models')
    output_path = join('results', dataset_name, 'output')
    os.makedirs(save_model_path, exist_ok=True)
    os.makedirs(output_path, exist_ok=True)
    config.save(join('results', dataset_name))
    return PathPack(log_path, save_model_path, output_path)


class PathPack(object):
    def __init__(self, log_path, save_model_path, output_path):
        self.log_path = log_path
        self.save_model_path = save_model_path
        self.output_path = output_path
