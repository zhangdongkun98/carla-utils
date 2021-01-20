
import os
from os.path import join
import time
from collections import namedtuple
from tensorboardX import SummaryWriter

PathPack = namedtuple('PathPack', ('log_path', 'save_model_path', 'output_path'))

def create_dir(config, model_name):
    '''
        create dir and save config
        Args:
            config: need to contain:
                config.description
    '''
    dataset_name = model_name + '/' + str(int(time.time())) + '----' + str(config.description)
    print('create dir: ', dataset_name)
    log_path = join('results', dataset_name, 'log')
    save_model_path = join('results', dataset_name, 'saved_models')
    output_path = join('results', dataset_name, 'output')
    os.makedirs(save_model_path, exist_ok=True)
    os.makedirs(output_path, exist_ok=True)
    with open(join('results', dataset_name, 'comments'), mode='w', encoding='utf-8') as _: pass
    config.save(join('results', dataset_name))

    logger = SummaryWriter(log_dir=log_path)
    logger.add_text('description', dataset_name, 0)
    return PathPack(log_path, save_model_path, output_path), logger

