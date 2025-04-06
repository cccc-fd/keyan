from plot_script import *
import csv
import numpy as np
from copy import deepcopy

# 文件名
filename = 'C:/Users/15017/Desktop/data.csv'

if __name__ == '__main__':

    with open(filename, 'rt') as f:
        # 读取文件
        csv_reader = csv.reader(f)
        # 索引
        index = 0
        mod = 5
        res = []
        res_item = []

        # 获取每一行
        for row in csv_reader:
            if index == mod:
                res.append(deepcopy(res_item))
                res_item.clear()
                index = 0
            # 当前每一行都是一个字符串类型的，转化为浮点型
            new_list = []  # 每一行数据
            for num in row:
                new_list.append(np.float64(num))
            # 第1个参数是float类型
            if index == 0:
                res_item.append(np.array(new_list))
            elif index == 1:
                res_item.append(np.float64(new_list[0]))
            # 第2个参数是int类型
            elif index == 2:
                res_item.append(int(new_list[0]))
            else:
                tmp = np.array(new_list)
                tmp = tmp.reshape(1, len(tmp))
                res_item.append(tmp)
            index += 1
        # 最后一组数据
        res.append(deepcopy(res_item))

    plot_all(res)
