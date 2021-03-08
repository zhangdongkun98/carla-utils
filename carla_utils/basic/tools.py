
import os
import numpy as np
import random
import copy

import torch
import torch.nn as nn


'''
https://stackoverflow.com/questions/15927755/opposite-of-numpy-unwrap
'''
def pi2pi(theta):
    if isinstance(theta, np.ndarray) or isinstance(theta, float):
        return pi2pi_numpy(theta)
    elif isinstance(theta, torch.Tensor):
        return pi2pi_numpy(theta)
    else: raise NotImplementedError
    

def pi2pi_numpy(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi

def pi2pi_tensor(theta):
    """
    Normalize the `theta` to have a value in [-pi, pi]

    Args:
        theta: Tensor of angles of shape N
    """
    TWO_PI = 2 * np.pi
    theta = torch.fmod(torch.fmod(theta, TWO_PI) + TWO_PI, TWO_PI)
    return torch.where(theta > np.pi, theta - TWO_PI, theta)


def np_dot(*args):
    res = args[0]
    for arg in args[1:]:
        res = np.dot(res, arg)
    return res

def int2onehot(index, length):
    '''
    Args:
        index: (batch_size,)
    return: numpy.array (length,)
    '''
    if isinstance(index, torch.Tensor):
        return nn.functional.one_hot(index.to(torch.int64), length).to(index.dtype)
    elif isinstance(index, np.ndarray):
        return np.eye(length)[index.reshape(-1)]
    elif isinstance(index, int):
        return np.eye(1, length, k=index)[0]
    else: raise NotImplementedError
    
def onehot2int(one_hot):
    if isinstance(one_hot, torch.Tensor):  ## will squeeze one dimension
        return torch.argmax(one_hot, dim=-1)
    else: raise NotImplementedError; return np.argmax(one_hot)


def prob2onehot(prob, length):
    '''
    Args:
        prob: (batch_size, length)
    '''
    if isinstance(prob, torch.Tensor):
        return nn.functional.one_hot(torch.argmax(prob, dim=1), length).to(prob.dtype)
    else: raise NotImplementedError



import matplotlib.pyplot as plt
def plotArrow2D(x, y, theta, length=1.0, width=0.5, fc='r', ec='k'):  # pragma: no cover
    plt.arrow(x, y, length * np.cos(theta), length * np.sin(theta),
              fc=fc, ec=ec, head_width=width, head_length=width)




def list_del(list_to_delete, delete_index_list):
    dil = copy.copy(delete_index_list)
    dil.sort()
    dil.reverse()
    for index in dil:
        del list_to_delete[index]

def flatten_list(input_list):
    """
    
    
    Args:
        input_list: 2-d list
    
    Returns:
        1-d list
    """
    
    output_list = []
    for i in input_list: output_list.extend(i)
    return output_list


def calculate_quadrant(point):
    """
    
    
    Args:
        point: contains attribute x, y
    
    Returns:
        int
    """

    if point.x > 0 and point.y > 0:
        quadrant = 1
    elif point.x < 0 and point.y > 0:
        quadrant = 2
    elif point.x < 0 and point.y < 0:
        quadrant = 3
    elif point.x > 0 and point.y < 0:
        quadrant = 4
    else:
        quadrant = 0
    return quadrant



def setup_seed(seed):
    random.seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True


