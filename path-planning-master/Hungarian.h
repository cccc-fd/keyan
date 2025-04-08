#ifndef MYPATHPLANNING_HUNGARIAN_H
#define MYPATHPLANNING_HUNGARIAN_H


#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>

#include "data.h"

using namespace std;

class Hungarian {
private:
    // 用于存储最后的坐标，源代码使用dict()数据类型存储
    unordered_map<int, int> dic;
    // 存放没有-1的行下标
    vector<int> rowIndex;
    // 存放没有-1的行中有-2的下标
    vector<int> colIndex;

public:
    // 传入一个值进来
    tuple<vector<tuple<int, int>>, double> hungrian(const vector<vector<double>>& values);

    // 画线
    double paintLine(const vector<vector<double>>& processValue2);

    int addRow(const vector<vector<double>>& processValue2);

    int addCol(const vector<vector<double>>& processValue2);

    // 寻找矩阵中哪一行的0个数最少
    int findLessZero(const vector<vector<double>>& processValues2);

    // 对col列所有值为0的元素赋值为-2
    void chang(int col, int row, vector<vector<double>>& processValue2);

    // 判断是否达到目的
    bool isMatchByRow(vector<vector<double>>& processValues2);
};


#endif //MYPATHPLANNING_HUNGARIAN_H
