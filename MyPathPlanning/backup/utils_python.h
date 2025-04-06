#ifndef MYPATHPLANNING_UTILS_PYTHON_H
#define MYPATHPLANNING_UTILS_PYTHON_H

#include <iostream>
#include <vector>

using namespace std;

// 一维数组差分
vector<double> numpy_diff(const vector<double>& origin);

// 二维数组按列差分
vector<vector<double>> numpy_diff(const vector<vector<double>>& origin);

vector<vector<double>> numpy_tile(const vector<double>& origin, const int& rows);

vector<double> numpy_linspace(double begin, double end, int num);


void print(const vector<int>& data);

void print(const vector<vector<int>>& data);

void print(const vector<double>& data);

void print(const vector<vector<double>>& data);

#endif //MYPATHPLANNING_UTILS_PYTHON_H
