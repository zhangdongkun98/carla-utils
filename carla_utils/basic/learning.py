
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



class nameddict(type):
    '''
        https://github.com/https://github.com/tonnydourado/nameddicttonnydourado/nameddict
    '''
    '''Metaclass (or class factory) for named dicts.
    Returns a class with a customized constructor,
    and dot-like access to dictionare items.
    Arguments:
    cls: implicit instance of the new class being generated.
    name: New classe's name.
    attrs: list of attributes for the new class.
    '''

    def __new__(cls, name, attrs=[]):
        def __getattribute__(self, name):
            class_ = dict.__getattribute__(self, '__class__')
            if name in super(class_, self).__getattribute__('keys')():
                return super(class_, self).__getitem__(name)
            else:
                return super(class_, self).__getattribute__(name)

        def __setattr__(self, name, value):
            class_ = dict.__getattribute__(self, '__class__')
            if name in super(class_, self).__getattribute__('keys')():
                super(class_, self).__setitem__(name, value)
            else:
                super(class_, self).__setattr__(name, value)
                super(class_, self).__setitem__(name, value)

        def __repr__(self):
            args = [
                '{key}={value}'.format(key=key, value=value)
                for key, value in self.items()
            ]
            args_str = '(' + u','.join(args) + ')'
            return self.__class__.__name__ + args_str

        dict_ = {
            '__getattribute__': __getattribute__,
            '__setattr__': __setattr__,
            '__repr__': __repr__
        }

        tpl = '''def __init__(self,{}):\n
                \tsuper(self.__class__,self).__init__()
                \tfor key,value in {}.items():\n
                \t\tsetattr(self,key,value)'''
        formal_args = ','.join(attrs)
        args_dict = 'dict(' + ','.join([i + '=' + i for i in attrs]) + ')'
        exec(tpl.format(formal_args, args_dict) in dict_)

        return super(nameddict, cls).__new__(cls, name, (dict,), dict_)

    def __init__(cls, name, attrs=[]):
        super(nameddict, cls).__init__(cls)


