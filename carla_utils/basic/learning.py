
import os
from os.path import join
import time

from tensorboardX import SummaryWriter


def create_dir(description, model_name):
    dataset_name = model_name + '/' + str(int(time.time())) + ' -- ' + description
    print('create dir: ', dataset_name)
    log_path = join('results', dataset_name, 'log')
    save_model_path = join('results', dataset_name, 'saved_models')
    output_path = join('results', dataset_name, 'output')
    os.makedirs(save_model_path, exist_ok=True)
    os.makedirs(output_path, exist_ok=True)
    logger = SummaryWriter(log_dir=log_path)
    return logger