#include "Hungarian.h"

/**
 * 使用匈牙利算法进行任务分配
 * @param cost_matrix 任务代价矩阵
 * @return
 */
tuple<vector<tuple<int, int>>, double> Hungarian::hungarian(const vector<vector<double>> &cost_matrix) {
    // 先初始化变量
    scheme.clear();
    rowIndex.clear();
    colIndex.clear();

    int row = (int) cost_matrix.size();
    int col = (int) cost_matrix[0].size();

    // 矩阵拷贝数据，准备变换
    vector<vector<double>> cost_matrix_copy(cost_matrix);
    // 标志位：
    bool flag = false;

    // 1.行归约：矩阵的每一行减去每一行的最小值
    for (int i = 0; i < cost_matrix_copy.size(); i++) {
        // 得到每一行的最小值
        double min_num = *min_element(cost_matrix_copy[i].begin(), cost_matrix_copy[i].end());
        // 每一行都的相应元素减去每一行的最小值
        for (int j = 0; j < cost_matrix_copy[i].size(); j++) {
            cost_matrix_copy[i][j] -= min_num;
        }
    }
    // 2.列归约：对矩阵的每一列减去每一列的最小值
    vector<double> col_list;
    for (int j = 0; j < col; j++) {
        col_list.clear();
        col_list.reserve(row);
        for (int i = 0; i < row; i++) {
            col_list.push_back(cost_matrix_copy[i][j]);
        }
        // 找到最小值
        double min_num = *min_element(col_list.begin(), col_list.end());
        for (int i = 0; i < row; i++) {
            cost_matrix_copy[i][j] -= min_num;
        }
    }

    // 3.试指派
    while (!trialAssignment(cost_matrix_copy)) {
        // 试指派失败，进入下一步：添加0元素
        // 试指派后，-1表示画圈0，-2表示被划去的0

        // 画线对行里没有-1的进行画线，对画线行里有-2的列画线，对画线列中有-1的画线
        // 不需要画线了，只需要找出要最小值的集合，求出最小值就行
        // 试指派失败后，画线
        double min_num = paintLine(cost_matrix_copy);
        // 还原数组：将置-1或置-2的归为0
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                if (cost_matrix_copy[i][j] == -1 || cost_matrix_copy[i][j] == -2) {
                    cost_matrix_copy[i][j] = 0;
                }
            }
        }
        // 需要判断画线数是否等于 length 若是等于，证明符合
        if (cost_matrix_copy.size() == (cost_matrix_copy.size() - rowIndex.size() + colIndex.size())) {
            if (trialAssignment(cost_matrix_copy)) {
                flag = true;
                break;
            }
        }

        for (int i: rowIndex) {
            for (int j = 0; j < cost_matrix_copy[i].size(); j++) {
                cost_matrix_copy[i][j] -= min_num;
            }
        }

        for (int i: colIndex) {
            for (int j = 0; j < cost_matrix_copy.size(); j++) {
                cost_matrix_copy[j][i] += min_num;
            }
        }
        // 清空变量数组
        colIndex.clear();
        rowIndex.clear();
    }

    double count = 0.0;
    if (flag) { // 指派成功
        for (auto &[key, value]: scheme) {
            count += cost_matrix[value - 1][key - 1];
        }
    } else {
        for (auto &[key, value]: scheme) {
            count += cost_matrix[key - 1][value - 1];
        }
    }

    // res需要先赋值
    vector<tuple<int, int>> res;
    res.reserve(this->scheme.size());
    for (const auto &[key, val]: this->scheme) {
        res.push_back(make_tuple(key, val));
    }
    // 根据第一个元素进行升序
    sort(res.begin(), res.end(), [](const tuple<int, int> &a, const tuple<int, int> &b) {
        return get<0>(a) < get<0>(b);
    });
    return {res, count};
}

/**
 * 画线
 * @param cost_matrix
 * @return
 */
double Hungarian::paintLine(const vector<vector<double>> &cost_matrix) {
    // 用的策略其实是：行有独立划掉列
    for (int i = 0; i < cost_matrix.size(); i++) {
        // 判断该行中是否存在-1，不存在则进入
        if (find(cost_matrix[i].begin(), cost_matrix[i].end(), -1) == cost_matrix[i].end()) {
            // 存储行
            this->rowIndex.push_back(i);
            // 遍历列
            for (int j = 0; j < cost_matrix[i].size(); j++) {
                // 存储值为-2的列
                if (cost_matrix[i][j] == -2) {
                    this->colIndex.push_back(j);
                }
            }
        }
    }
    while (true) {
        int n = this->addCol(cost_matrix) + this->addRow(cost_matrix);
        if (n == 0) {
            break;
        }
    }
    // 从没有画线的数字中找出最小值
    bool flag = true;
    double min_num = 0;
    for (int i: rowIndex) {
        for (int j = 0; j < cost_matrix[i].size(); j++) {
            // 除去画线列
            if (find(colIndex.begin(), colIndex.end(), j) == colIndex.end()) {
                if (flag) {
                    min_num = cost_matrix[i][j];
                    flag = false;
                }
                if (min_num > cost_matrix[i][j]) {
                    min_num = cost_matrix[i][j];
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
 * @param cost_matrix
 * @return
 */
int Hungarian::addCol(const vector<vector<double>> &cost_matrix) {
    int cnt = 0;
    for (int i: rowIndex) {
        for (int j = 0; j < cost_matrix[i].size(); j++) {
            if (find(colIndex.begin(), colIndex.end(), j) == colIndex.end() && cost_matrix[i][j] == -2) {
                cnt++;
                colIndex.push_back(j);
            }
        }
    }
    return cnt;
}

/**
 * 找打代价矩阵中0元素最少的行
 * @param cost_matrix 成本矩阵
 * @return 返回对应的行数
 */
int Hungarian::findLessZero(const vector<vector<double>> &cost_matrix) {
    int min_num = INT_MAX;
    // 返回值
    int res = -1;
    // 循环遍历，找到0最少的行
    for (int i = 0; i < cost_matrix.size(); i++) {
        // 计数器count
        int cnt = 0;
        // 标志某一行是否有0
        bool haveZero = false;
        for (int j = 0; j < cost_matrix[i].size(); j++) {
            if (cost_matrix[i][j] == 0) {
                cnt++;
                haveZero = true;
            }
        }
        // 若最小值个数大于当前最小值，则变换
        if (haveZero && min_num > cnt) {
            min_num = cnt;
            res = i;
        }
    }
    return res;
}

/**
 * 将画圈0所在行、所在列的其它0元素划去
 * 实际做法：对画圈0所在行、所在列的其它0元素置为-2
 * @param col 画圈0所在的列
 * @param row 画圈0所在的行
 * @param cost_matrix 代价矩阵 使用引用，需要将结果返回给调用者
 */
void Hungarian::change(int col, int row, vector<vector<double>> &cost_matrix) {
    // 所在列0元素置-2，表示划去
    for (int i = 0; i < cost_matrix.size(); i++) {
        if (cost_matrix[i][col] == 0) {
            cost_matrix[i][col] = -2;
        }
    }
    // 所在行0元素置为-2，表示划去
    for (int i = 0; i < cost_matrix[row].size(); i++) {
        if (cost_matrix[row][i] == 0) {
            cost_matrix[row][i] = -2;
        }
    }
    // -1表示独立0元素，十字交叉位置
    cost_matrix[row][col] = -1;
}

/**
 * 试指派
 * @param cost_matrix 代价矩阵 使用引用，需要将结果返回给调用者
 * @return 试指派是否成功
 */
bool Hungarian::trialAssignment(vector<vector<double>> &cost_matrix) {
    // 独立元素0个
    int onlyZero = 0;
    // 从0最少的一行开始进行变换
    while (true) {
        // 找到0最少的行
        int lessZeroIndex = findLessZero(cost_matrix);
        // 没有0元素直接break，不需要下一步操作
        if (lessZeroIndex == -1) break;

        for (int j = 0; j < cost_matrix[lessZeroIndex].size(); j++) {
            if (cost_matrix[lessZeroIndex][j] == 0) {
                // 存储结果，表示：第 i 架无人机执行第 j 项任务
                this->scheme[lessZeroIndex + 1] = j + 1;
                onlyZero++;
                // 将画圈0所在行、所在列的其它0元素置为-2，表示划去，画圈0置为-1，表示画圈
                this->change(j, lessZeroIndex, cost_matrix);
                break;
            }
        }
    }
    // 没有达到目的，清空坐标
    if (onlyZero != cost_matrix.size()) {
        this->scheme.clear();
        return false;
    } else {
        return true;
    }
}
