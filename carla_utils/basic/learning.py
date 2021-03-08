
import os
from os.path import join
import time
from collections import namedtuple
from tensorboardX import SummaryWriter, GlobalSummaryWriter

PathPack = namedtuple('PathPack', ('log_path', 'save_model_path', 'output_path'))

def create_dir(config, model_name):
    '''
        create dir and save config
        Args:
            config: need to contain:
                config.description: str
                config.method: str
                config.eval: bool
    '''
    if config.eval: model_name = '000-eval/' + model_name
    dataset_name = model_name + '/' + config.method + '--' + str(int(time.time())) + '----' + str(config.description)
    print('create dir: ', dataset_name)
    log_path = join('results', dataset_name, 'log')
    save_model_path = join('results', dataset_name, 'saved_models')
    output_path = join('results', dataset_name, 'output')
    os.makedirs(save_model_path, exist_ok=True)
    os.makedirs(output_path, exist_ok=True)
    with open(join('results', dataset_name, 'comments'), mode='w', encoding='utf-8') as _: pass
    config.save(join('results', dataset_name))

    writer = Writer(log_dir=log_path, comment=dataset_name)
    return PathPack(log_path, save_model_path, output_path), writer


class Writer(SummaryWriter):
    def __init__(self, **kwargs):
        super(Writer, self).__init__(**kwargs)

        description = kwargs['comment']
        self.add_text('description', description, 0)

        file_dir, file_name = os.path.split(self.file_writer.event_writer._ev_writer._file_name)
        dir_name = file_name.split('tfevents.')[-1].split('.')[0] + '--log'

        self.data_dir = join(file_dir, dir_name)
        os.makedirs(self.data_dir)

        self.data_cache = dict()
    
    
    def _clear_cache(self, tag):
        self.data_cache[tag]['count'] = 0
        self.data_cache[tag]['data'] = ''
    
    def _write_cache_to_disk(self, tag):
        file_path = join(self.data_dir, tag+'.txt')
        file_dir, _ = os.path.split(file_path)
        os.makedirs(file_dir, exist_ok=True)
        with open(file_path, mode='a') as f:
            f.write(self.data_cache[tag]['data'])
        
        self._clear_cache(tag)


    def add_scalar(self, *args):
        res = super().add_scalar(*args)

        tag, scalar_value, global_step = args[0], args[1], args[2]

        if tag not in self.data_cache:
            self.data_cache[tag] = dict()
            self._clear_cache(tag)
        self.data_cache[tag]['count'] += 1
        self.data_cache[tag]['data'] += str(global_step) + ' ' + str(scalar_value) + '\n'
        
        if self.data_cache[tag]['count'] % 300 == 0:
            self._write_cache_to_disk(tag)
        return res


    def close(self):
        res = super().close()

        for tag in self.data_cache:
            self._write_cache_to_disk(tag)
        return res

