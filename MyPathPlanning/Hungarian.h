#ifndef MYPATHPLANNING_HUNGARIAN_H
#define MYPATHPLANNING_HUNGARIAN_H


#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>

using namespace std;

class Hungarian {
private:
    // 任务分配方案
    unordered_map<int, int> scheme;
    // 存放没有-1的行下标
    vector<int> rowIndex;
    // 存放没有-1的行中有-2的下标
    vector<int> colIndex;


    // 画线
    double paintLine(const vector<vector<double>>& cost_matrix);

    int addRow(const vector<vector<double>>& processValue2);

    int addCol(const vector<vector<double>>& cost_matrix);

    // 找打代价矩阵中0元素最少的行
    int findLessZero(const vector<vector<double>>& cost_matrix);

    // 对col列所有值为0的元素赋值为-2
    void change(int col, int row, vector<vector<double>>& cost_matrix);

    // 判断是否达到目的
    bool trialAssignment(vector<vector<double>>& cost_matrix);

public:
    // 传入一个值进来
    tuple<vector<tuple<int, int>>, double> hungarian(const vector<vector<double>>& cost_matrix);
};


#endif //MYPATHPLANNING_HUNGARIAN_H
