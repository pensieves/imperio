import numpy as np


def lastargmatch(arr, match):
    arg_pos = np.array(np.where(arr == match))
    arg_pos = np.ravel_multi_index(arg_pos, arr.shape)
    arg_pos = np.unravel_index(arg_pos.max(), arr.shape)
    return arg_pos


def lastargmax(arr):
    return lastargmatch(arr, arr.max())


def lastargmin(arr):
    return lastargmatch(arr, arr.min())
