
import os, sys
from os.path import join
import glob
import json
from numpy.lib.arraysetops import isin
import yaml
import inspect


def load_carla_standard():
    try:
        server_path = os.environ['CARLAPATH']
    except:
        print('run this in shell:\n    echo "export CARLAPATH=/your/carla/server/path" >> ~/.bashrc')
        exit(0)
    try:
        sys.path.append(server_path+'/PythonAPI/carla')
        sys.path.append(glob.glob(server_path+'/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    except:
        print('Fail to load carla library')
        print('run this in shell:\n    echo "export CARLAPATH=/your/carla/server/path" >> ~/.bashrc')
        exit(0)


def load_carla():
    basic_path = os.path.split(os.path.split(__file__)[0])[0]
    path = join(basic_path, 'carla_api')
    try:
        sys.path.append(path+'/carla')
        carla_path = glob.glob(path+'/carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0]
        sys.path.append(carla_path)
    except:
        print('Fail to load carla library')


def printVariable(name, value):
    print(name + ': ' + str(value))


def parse_json_file(file_path):
    if not os.path.exists(file_path):
        raise RuntimeError("Could not read json file from {}".format(file_path))
    json_dict = None
    with open(file_path) as handle:
        json_dict = json.loads(handle.read())
    return json_dict

def parse_yaml_file(file_path):
    data = None
    with open(file_path) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    return data

def parse_yaml_file_unsafe(file_path):
    data = None
    with open(file_path) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    config = YamlConfig(data, file_path)
    # config.set_path(os.path.abspath(file_path))
    return config

class YamlConfig(object):
    def __init__(self, args_dict, file_path):
        self._file_path = file_path
        for key in args_dict:
            if isinstance(args_dict[key], type(dict())):
                setattr(self, key, YamlConfig(args_dict[key], file_path))
            else:
                setattr(self, key, args_dict[key])
    
    def __getattribute__(self, name):
        try:
            return object.__getattribute__(self, name)
        except:
            e = 'yaml file \'{}\' has not attribute \'{}\''.format(self._file_path, name)
            raise AttributeError(e)
    
    def get(self, key, default):
        result = default
        if hasattr(self, key):
            result = getattr(self, key)
        return result
    
    def update(self, config):
        """
        
        Args:
            config: YamlConfig or argparse
        
        Returns:
            None
        """

        block_words = self._get_block_words()

        for attribute in dir(config):
            if attribute in block_words:
                if not isinstance(config, YamlConfig): print('[YamlConfig] ignore attribute: ', attribute)
                continue
            if not attribute.startswith('_'):
                setattr(self, attribute, getattr(config, attribute))
        return
    
    def _get_block_words(self):
        block_words = [attr for attr in dir(YamlConfig) if callable(getattr(YamlConfig, attr)) and not attr.startswith('_')]
        return block_words
    
    # def set_path(self, path):
    #     self.path = path
    
    def convert_to_dict(self):
        block_words = self._get_block_words()
        result = dict()
        for attribute in dir(self):
            if attribute in block_words: continue
            if not attribute.startswith('_'):
                value = getattr(self, attribute)
                if isinstance(value, YamlConfig):
                    result[attribute] = value.convert_to_dict()
                else: result[attribute] = value
        return result

    def save(self, path):
        config_dict = self.convert_to_dict()
        with open(join(path, 'config.yaml'), 'w', encoding='utf-8') as f:
            yaml.dump(data=config_dict, stream=f, allow_unicode=True)
        return



class Singleton(object):
    _instance = None

    def __new__(cls, *args, **kw):
        if not cls._instance:
            cls._instance = super(Singleton, cls).__new__(cls)  
        return cls._instance


def debug(info, info_type='debug'):
    if info_type == 'error':
        print('\033[1;31m ERROR:', info, '\033[0m')
    elif info_type == 'success':
        print('\033[1;32m SUCCESS:', info, '\033[0m')
    elif info_type == 'warning':
        print('\033[1;34m WARNING:', info, '\033[0m')
    elif info_type == 'debug':
        print('\033[1;35m DEBUG:', info, '\033[0m')
    else:
        print('\033[1;36m MESSAGE:', info, '\033[0m')


def mkdir(path):
    if not os.path.exists(path):
        os.makedirs(path)


def retrieve_name(var):
    callers_local_vars = inspect.currentframe().f_back.f_locals.items()
    return [var_name for var_name, var_val in callers_local_vars if var_val is var][0]


def pdb_set():
    ### TODO
    import pdb
    import rlcompleter
    pdb.Pdb.complete = rlcompleter.Completer(locals()).complete
    pdb.set_trace()

