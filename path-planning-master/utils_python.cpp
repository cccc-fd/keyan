#include "utils_python.h"

/**
 * 一维数组差分
 * @param origin    原数组
 * @return          差分数组
 */
vector<double> numpy_diff(const vector<double> &origin) {
    vector<double> res;
    // 预分配空间，增加效率
    res.reserve(origin.size() - 1);
    for (int i = 0; i < origin.size() - 1; i++) {
        res.emplace_back(origin[i + 1] - origin[i]);
    }
    return res;
}

/**
 * 二维数组元素按列差分：后一列减去前一列
 * @param origin  原数组
 * @return      做好减法的数组
 */
vector<vector<double>> numpy_diff(const vector<vector<double>> &origin) {
    vector<vector<double>> res(origin.size(), vector<double>(origin[0].size() - 1));
    for (int i = 0; i < origin.size(); i++) {
        for (int j = 0; j < origin[i].size() - 1; j++) {
            res[i][j] = origin[i][j + 1] - origin[i][j];
        }
    }
    return res;
}


vector<double> numpy_linspace(double begin, double end, int num) {
    vector<double> res;
    // 预分配空间
    res.reserve(num);
    for (int i = 0; i < num; i++) {
        res.emplace_back((end - begin) / (num - 1) * i);
    }
    return res;
}


void print(const vector<int> &data) {
    for (auto e: data) cout << e << "\t\t";
    cout << endl;
}

void print(const vector<vector<int>> &data) {
    for (const auto &row: data) {
        for (const auto &e: row) {
            cout << e << "\t\t";
        }
        cout << endl;
    }
}

void print(const vector<double> &data) {
    for (auto e: data) cout << e << "\t\t";
    cout << endl;
}

void print(const vector<vector<double>> &data) {
    for (const auto &row: data) {
        for (const auto &e: row) {
            cout << e << "\t\t";
        }
        cout << endl;
    }
}