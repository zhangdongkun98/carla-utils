
import os, sys
from os.path import join
import glob
import json
import yaml
import inspect


def load_carla_standard(path):
    try:
        sys.path.append(path+'/PythonAPI')
        sys.path.append(glob.glob(path+'/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    except:
        print('Fail to load carla library')

basic_path = os.path.split(os.path.split(__file__)[0])[0]
def load_carla():
    path = join(basic_path, 'carla_api/0.9.9.4')
    try:
        sys.path.append(path+'/carla')
        sys.path.append(glob.glob(path+'/carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
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
    return YamlConfig(data, file_path)

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