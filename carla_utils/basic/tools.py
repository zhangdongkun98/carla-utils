
import numpy as np
import torch
import torch.nn as nn
import copy

'''
https://stackoverflow.com/questions/15927755/opposite-of-numpy-unwrap
'''
def pi2pi(theta, theta0=0.0):
    # while(theta > np.pi + theta0):
    #     theta = theta - 2.0 * np.pi
    # while(theta < -np.pi + theta0):
    #     theta = theta + 2.0 * np.pi
    # return theta
    return (theta + np.pi) % (2 * np.pi) - np.pi

def np_dot(*args):
    res = args[0]
    for arg in args[1:]:
        res = np.dot(res, arg)
    return res

def int2onehot(index : int, length):
    '''
    Args:
        index: (batch_size,)
    return: numpy.array (length,)
    '''
    if isinstance(index, torch.Tensor):
        return nn.functional.one_hot(index.to(torch.int64), length).to(index.dtype)
    elif isinstance(index, np.ndarray):
        raise NotImplementedError
        return np.eye(1, length, k=index)[0]
    elif isinstance(index, int):
        return np.eye(1, length, k=index)[0]
    else: raise NotImplementedError
    
def onehot2int(one_hot):
    return np.argmax(one_hot)


def prob2onehot(prob, length):
    '''
    Args:
        prob: (batch_size, length)
    '''
    if isinstance(prob, torch.Tensor):
        return nn.functional.one_hot(torch.argmax(prob, dim=1), length).to(prob.dtype)
    else: raise NotImplementedError



def list_del(list_to_delete, delete_index_list):
    dil = copy.copy(delete_index_list)
    dil.sort()
    dil.reverse()
    for index in dil:
        del list_to_delete[index]


import matplotlib.pyplot as plt
def plotArrow2D(x, y, theta, length=1.0, width=0.5, fc='r', ec='k'):  # pragma: no cover
    plt.arrow(x, y, length * np.cos(theta), length * np.sin(theta),
              fc=fc, ec=ec, head_width=width, head_length=width)


