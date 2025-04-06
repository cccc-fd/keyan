from scipy.interpolate import interp1d
import numpy as np


def get_interpolation_nums(nums, point_num):
    '''
    :param nums:        原始数据
    :param point_num:   插值点个数
    :return:
    '''
    # 插值
    size = len(nums)
    x = np.linspace(0, 1, size)
    y = np.array(nums)
    # 生成的插值函数
    f = interp1d(x, y, kind='cubic', assume_sorted=True)

    x_new = np.linspace(0, 1, point_num)
    y_new = f(x_new)

    return y_new.tolist()
