#include "Hungarian.h"

tuple<vector<tuple<int, int>>, double> Hungarian::hungrian(const vector<vector<double>> &values) {
    // 先初始化变量
    dic.clear();
    rowIndex.clear();
    colIndex.clear();

    // 二维数组，矩阵拷贝数据，准备变换
    vector<vector<double>> values_copy(values);
    // 标志位：
    bool flag = false;

    // 1.对数组的每一行减去每一行的最小值
    for (int i = 0; i < values_copy.size(); i++) {
        // 得到每一行的最小值
        double min_num = *min_element(values_copy[i].begin(), values_copy[i].end());
        // 每一行都的相应元素减去每一行的最小值
        for (int j = 0; j < values_copy[i].size(); j++) {
            values_copy[i][j] -= min_num;
        }
    }
    // 2.对对数组的每一列减去每一列的最小值
    vector<double> col_list;
    int row = values_copy.size();
    int col = values_copy[0].size();
    for (int j = 0; j < col; j++) {
        col_list.clear();
        for (int i = 0; i < row; i++) {
            col_list.push_back(values_copy[i][j]);
        }
        // 找到最小值
        double min_num = *min_element(col_list.begin(), col_list.end());
        for (int i = 0; i < row; i++) {
            values_copy[i][j] -= min_num;
        }
    }
    // 3.当进行到这一步时候有的成本矩阵已经达到了我们想要的目的了，需要进行判断是否达到
    while (!isMatchByRow(values_copy)) {
        // 画线对行里没有-1的进行画线，对画线行里有-2的列画线，对画线列中有-1的画线
        // 不需要画线了，只需要找出要最小值的集合，求出最小值就行
        double min_num = paintLine(values_copy);
        // 还原数组：将置-1或置-2的归为0
        for (int i = 0; i < values_copy.size(); i++) {
            for (int j = 0; j < values_copy[i].size(); j++) {
                if (values_copy[i][j] == -1 || values_copy[i][j] == -2) {
                    values_copy[i][j] = 0;
                }
            }
        }
        // 需要判断画线数是否等于 length 若是等于，证明符合
        if (values_copy.size() == (values_copy.size() - rowIndex.size() + colIndex.size())) {
            if (isMatchByRow(values_copy)) {
                flag = true;
                break;
            }
        }

        for (int i: rowIndex) {
            for (int j = 0; j < values_copy[i].size(); j++) {
                values_copy[i][j] -= min_num;
            }
        }

        for (int i: colIndex) {
            for (int j = 0; j < values_copy.size(); j++) {
                values_copy[j][i] += min_num;
            }
        }
        // 清空变量数组
        colIndex.clear();
        rowIndex.clear();
    }

    double count = 0.0;
    if (flag) {
        for (auto &[key, value]: dic) {
            count += values[value - 1][key - 1];
        }
    } else {
        for (auto &[key, value]: dic) {
            count += values[key - 1][value - 1];
        }
    }

    // res需要先赋值
    vector<tuple<int, int>> res;
    for (const auto &[key, val]: this->dic) {
        res.push_back(make_tuple(key, val));
    }
    // 排序
    sort(res.begin(), res.end(), [](const tuple<int, int> &a, const tuple<int, int> &b) {
        return get<0>(a) < get<0>(b);
    });
    return {res, count};
}

/**
 * 画线
 * @param processValue2
 * @return
 */
double Hungarian::paintLine(const vector<vector<double>> &processValue2) {
    for (int i = 0; i < processValue2.size(); i++) {
        // 判断该行中是否存在-1，不存在则进入
        if (find(processValue2[i].begin(), processValue2[i].end(), -1) == processValue2[i].end()) {
            // 存储行
            this->rowIndex.push_back(i);
            // 遍历列
            for (int j = 0; j < processValue2[i].size(); j++) {
                // 存储值为-2的列
                if (processValue2[i][j] == -2) {
                    this->colIndex.push_back(j);
                }
            }
        }
    }
    while (true) {
        int n = this->addCol(processValue2) + this->addRow(processValue2);
        if (n == 0) {
            break;
        }
    }
    // 从没有画线的数字中找出最小值
    bool flag = true;
    double min_num = 0;
    for (int i: rowIndex) {
        for (int j = 0; j < processValue2[i].size(); j++) {
            // i 不在里面，即列不在里面
            if (find(colIndex.begin(), colIndex.end(), j) == colIndex.end()) {
                if (flag) {
                    min_num = processValue2[i][j];
                    flag = false;
                }
                if (min_num > processValue2[i][j] + eps) {
                    min_num = processValue2[i][j];
                }
            }
        }
    }
    // 返回最小值
    return min_num;
}

/**
 *
 * @param processValue2
 * @return
 */
int Hungarian::addRow(const vector<vector<double>> &processValue2) {
    int cnt = 0;
    for (int i: colIndex) {
        for (int j = 0; j < processValue2[i].size(); j++) {
            if (find(rowIndex.begin(), rowIndex.end(), j) == rowIndex.end()
                && (processValue2[j][i] == -1)) { // 等于-1，精度处理
                cnt++;
                rowIndex.push_back(j);
            }
        }
    }
    return cnt;
}

/**
 *
 * @param processValue2
 * @return
 */
int Hungarian::addCol(const vector<vector<double>> &processValue2) {
    int cnt = 0;
    for (int i: rowIndex) {
        for (int j = 0; j < processValue2[i].size(); j++) {
            if (find(colIndex.begin(), colIndex.end(), j) == colIndex.end() && processValue2[i][j] == -2) {
                cnt++;
                colIndex.push_back(j);
            }
        }
    }
    return cnt;
}

/**
 * 寻找矩阵中哪一行的0个数最少
 * @param processValues2 成本矩阵
 * @return 返回对应的行数
 */
int Hungarian::findLessZero(const vector<vector<double>> &processValues2) {
    // 这里n用于保留一个最小值，不过此处初始化processValues2.size()最大
    int min_num = INT_MAX;
    // 返回值
    int res = -1;
    // 循环遍历，找到0最少的行
    for (int i = 0; i < processValues2.size(); i++) {
        // 计数器count
        int cnt = 0;
        // 标志某一行是否有0
        bool zeroFlag = false;
        for (int j = 0; j < processValues2[i].size(); j++) {
            if (processValues2[i][j] == 0) {
                cnt++;
                zeroFlag = true;
            }
        }
        // 若最小值个数大于当前最小值，则变换
        if (zeroFlag && min_num > cnt) {
            min_num = cnt;
            res = i;
        }
    }
    return res;
}

/**
 * 对col列所有值为0的元素赋值为-2
 * @param col
 * @param row
 * @param processValue2
 */
void Hungarian::chang(int col, int row, vector<vector<double>> &processValue2) {
    for (int i = 0; i < processValue2.size(); i++) {
        if (processValue2[i][col] == 0) {
            processValue2[i][col] = -2;
        }
    }
    // 对row行所有值为0的元素赋值为-2
    for (int i = 0; i < processValue2[row].size(); i++) {
        if (processValue2[row][i] == 0) {
            processValue2[row][i] = -2;
        }
    }
    // -1表示独立0元素，十字交叉位置
    processValue2[row][col] = -1;
}

/**
 * 判断是否达到目的
 * @param processValues2
 * @return
 */
bool Hungarian::isMatchByRow(vector<vector<double>> &processValues2) {
    // 独立元素0个
    int onlyZero = 0;
    // 标志位：若为真，则达到要求，输出；若为假，返回false
    bool flag = false;

    // 从0最少的一行开始进行变换
    while (true) {
        // 找到0最少的行
        int lessZeroIndex = findLessZero(processValues2);
        if (lessZeroIndex == -1) {
            break;
        }
        for (int j = 0; j < processValues2[lessZeroIndex].size(); j++) {
            if (processValues2[lessZeroIndex][j] == 0) {
                // 存储坐标？
                this->dic[lessZeroIndex + 1] = j + 1;
                onlyZero++;

                // i: 列数 lessZeroIndex: 行 processValues2: 矩阵
                this->chang(j, lessZeroIndex, processValues2);
                break;
            }
        }
    }
    // 没有达到目的，清空坐标
    if (onlyZero != processValues2.size()) {
        this->dic.clear();
    } else {
        flag = true;
    }
    // 返回标志
    return flag;
}
