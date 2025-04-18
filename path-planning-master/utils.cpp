#include <cmath>
//#include <Python.h>
//#include <libInterpolate/Interpolate.hpp>

#include "utils.h"

/**
 * 球：获得切面圆半径
 * @param x：x
 * @param y: y
 * @param z: z
 * @param r: 半径
 * @param h: 高度
 * @return 返回值先使用元组接受，后期确定后应当定义结构体或者类来封装数据
 */
tuple<double, double, double> get_circle(int x, int y, int z, int r, int h) {
    double real_r = sqrt(r * r - (h - z) * (h - z));
    return {x, y, real_r};
}

/**
 * 山体：抽象为圆锥
 * @param x
 * @param y
 * @param z
 * @param r
 * @param h
 * @return
 */
tuple<double, double, double> get_mountain_circle(int x, int y, int z, int r, int h) {
    double real_r = r * ((z - (h - z)) / static_cast<double>(z));
    return {x, y, real_r};
}

/**
 * 天气：抽象为圆柱
 * @param x
 * @param y
 * @param z
 * @param r
 * @param h
 * @return
 */
tuple<double, double, double> get_weather_circle(int x, int y, int z, int r, int h) {
    return {x, y, r};
}

/**
 * 多边形
 * @param V 多边形的端点，应当是从右下角开始的，逆时针
 * @return 多边形的中间点
 */
tuple<double, double> centroid(const vector<vector<int>> &V) {
    // 端点个数
    int n_pts = V.size();

    double xc = 0.0;
    double yc = 0.0;
    double A = 0.0;

    for (int i = 0; i < n_pts; ++i) {
        int index = ((i - 1) + n_pts) % n_pts;
        double _d = V[index][0] * V[i][1] - V[i][0] * V[index][1];
        xc += (V[index][0] + V[i][0]) * _d;
        yc += (V[index][1] + V[i][1]) * _d;
        A += _d;
    }

    A = A / 2.0;
    xc = xc / (6.0 * A);
    yc = yc / (6.0 * A);
    return {xc, yc};
}

/**
 * 得到所有的障碍信息，即将障碍坐标转化为具体的切面圆或者其他类型
 * @return 障碍切面数组
 */
vector<Obstacle> get_all_obs() {
    // 障碍切面信息集合
    vector<Obstacle> obs;
    double x, y;
    double real_r;
    // 遍历map，障碍信息
    for (auto [key, val]: obstacle) {
        if (key == "radar" || key == "gun" || key == "missile") {
            for (auto data: val) {
                tie(x, y, real_r) = get_circle(data[0], data[1], data[2], data[3], start[2]);
                // 向obs中追加数据
                obs.push_back({"cir", x, y, real_r});
            }
        } else if (key == "mountain") {
            for (auto data: val) {
                tie(x, y, real_r) = get_mountain_circle(data[0], data[1], data[2], data[3], start[2]);
                obs.push_back({"cir", x, y, real_r});
            }
        } else if (key == "weather") {
            for (auto data: val) {
                tie(x, y, real_r) = get_weather_circle(data[0], data[1], data[2], data[3], start[2]);
                obs.push_back({"cir", x, y, real_r});
            }
        } else if (key == "no_fly") {
            for (const auto &data: val) {
                // 用于存储不规则多边形的各边信息
                vector<vector<int>> V;
                for (int i = 0; i < data.size(); i += 2) {
                    vector<int> v1 = {data[i], data[i + 1]};
                    V.push_back(v1);
                }
                tie(x, y) = centroid(V);
                obs.push_back({"con", x, y, 0.0, 0.0, V});
            }
        }
    }
    return obs;
}


/**
 * 设置粒子的初始状态，起点到目标点的简单直线
 * @param _start
 * @param _goal
 * @param n_pts
 * @return
 */
vector<double> build_Xinit(vector<double> &_start, vector<double> &_goal, int n_pts) {
    // 起点
    double xs = _start[0];
    double ys = _start[1];
    // 目标
    double xg = _goal[0];
    double yg = _goal[1];

    // 预置空间
    vector<double> Px(n_pts + 2);
    vector<double> Py(n_pts + 2);

    // 生成一个xs -> xg 和 ys -> yg 的序列
    for (int i = 0; i < n_pts + 2; i++) {
//        Px[i] = xs + (xg - xs) * (i + 1) / (n_pts + 2);
        Px[i] = xs + (xg - xs) / (n_pts + 2 - 1) * i;
        Py[i] = ys + (yg - ys) / (n_pts + 2 - 1) * i;
    }

    // 首尾元素不取
    vector<double> Xinit(Px.begin() + 1, Px.end() - 1);
    Xinit.insert(Xinit.end(), Py.begin() + 1, Py.end() - 1);

    return Xinit;
}




/*
 *********************************** main **********************************
 *
 * */

/**
 * 增加圆形障碍
 * @param x
 * @param y
 * @param r
 * @param Kv
 * @return
 */
Obstacle add_circle_obs(double x, double y, double r, double Kv) {
    return {"Circle", x, y, r, Kv};
}

/**
 * 添加禁飞区
 * @param x
 * @param y
 * @param V
 * @param kv
 * @return
 */
Obstacle add_convex(double x, double y, vector<vector<int>> V, double kv) {
    return {"Convex", x, y, 0.0, kv, std::move(V)};
}


/**
 * 计算起飞点与目标点的直线距离
 * @param starts
 * @param targets
 * @return
 */
vector<vector<double>> calcu_line_start_target(vector<vector<int>> &starts, vector<vector<int>> &targets) {
    // 4行4列
    vector<vector<double>> res(starts.size(), vector<double>(starts.size()));
    for (int i = 0; i < starts.size(); i++) {
        for (int j = 0; j < targets.size(); j++) {
            res[i][j] = sqrt(pow(starts[i][0] - targets[j][0], 2) + pow(starts[i][1] - targets[j][1], 2));
        }
    }
    return res;
}

/*
 ************************************** run_example ********************************************
 * */

/**
 * 计算实现全覆盖目标的长度   l2
 * @param cycle_count 往复次数
 * @param goal_length 目标长度
 * @return
 */
double get_l2(const int &cycle_count, const double &goal_length) {
    return cycle_count * goal_length;
}

/**
 * 得到返航的长度l3
 * @param _goal         目标位置（4端点）
 * @param l1            到达目标位置的长度
 * @param cycle_count   往复次数
 * @param goal_length   目标长度
 * @return
 */
double get_l3(vector<int> _goal, double l1, int cycle_count, double goal_length) {
    double l3 = 0.0;
    if (cycle_count == 0) { // 点型目标
        l3 = l1;
    } else if (cycle_count == 1) { // 线型目标
        l3 = goal_length + l1;
    } else if (cycle_count > 1) { // 面型目标
        if (cycle_count % 2 == 0) {
            l3 = l1 + _goal[10] - _goal[1];
        } else {
            l3 = l1 +
                 sqrt((_goal[0] - _goal[6]) * (_goal[0] - _goal[0]) + (_goal[1] - _goal[7]) * (_goal[1] - _goal[7]));
        }
    }
    return l3;
}

/**
 * 计算无人机探查目标需要往复几次实现全覆盖   目标长度
 * @param F         无人机侦察角
 * @param h         无人机飞行高度
 * @param _goal     目标数据：{西南、东南、东北、西北}
 * @return          {往复次数，路径长度}
 */
tuple<int, double> calcu_cycle_count(double F, int h, vector<int> _goal) {
    // 侦察视角宽度
    double uav_d = 2 * h * tan(F / 2);

    int A1 = _goal[4] - _goal[1];
    int B1 = _goal[0] - _goal[3];
    double C1 = _goal[3] * (_goal[1] - _goal[4]) - _goal[4] * (_goal[0] - _goal[3]);

    int A2 = _goal[10] - _goal[7];
    int B2 = _goal[6] - _goal[9];
    double C2 = _goal[9] * (_goal[7] - _goal[10]) - _goal[10] * (_goal[6] - _goal[9]);

    if (A1 == 0 && B1 != 0) {
        C2 = (double) B2 / B1 * C2;
    } else if (B1 == 0 && A1 != 0) {
        C2 = (double) A1 / A2 * C2;
    }
    double goal_d_1 = fabs(C2 - C1) / sqrt(A1 * A1 + B1 * B1);


    A1 = _goal[10] - _goal[1];
    B1 = _goal[0] - _goal[9];
    C1 = _goal[9] * (_goal[1] - _goal[10]) - _goal[10] * (_goal[0] - _goal[9]);
    A2 = _goal[7] - _goal[4];
    B2 = _goal[3] - _goal[6];
    C2 = _goal[6] * (_goal[4] - _goal[7]) - _goal[7] * (_goal[3] - _goal[6]);

    if (A1 == 0 && B1 != 0) {
        C2 = (double) B2 / B1 * C2;
    } else if (B1 == 0 && A1 != 0) {
        C2 = (double) A1 / A2 * C2;
    }
    double goal_d_2 = abs(C2 - C1) / sqrt(A1 * A1 + B1 * B1);

    double goal_d = goal_d_1 < goal_d_2 ? goal_d_1 : goal_d_2;
    double goal_length = goal_d_1 < goal_d_2 ? goal_d_2 : goal_d_1;

    // 判断double类型的数据，使用一个极小值 eps = 1e-6 辅助
    int cycle_count = 0;
    if (uav_d > goal_d + eps) {
        cycle_count = 1;
    } else {
        cycle_count = (int) ceil(goal_d / uav_d);
    }
    return {cycle_count, goal_length};
}


/*
 ********************************** PathPlanning begin *******************************
 * */
/**
 * 返回最小化的函数，即当没有任何障碍冲突时的路径长度。
 * PathPlanning USE
 * calc_path_length for vector<double>
 * @param agent_pos
 * @param args
 * @return
 */
tuple<double, int, vector<double>, vector<double>>
calc_path_length(vector<double> agent_pos, Args &args) {
    // 获取参数数据
    int Xs = args.starts_odv[0];
    int Ys = args.starts_odv[1];
    int Xg = args.goals_odv[0];
    int Yg = args.goals_odv[1];
    vector<Obstacle> obs = args.obs;
    int pointNums = args.interpolationPointNums;
    string _finterp = args.interpolationKind;

    // 这种情况下是较为特殊的，最后算一维数组
    int n_pop = 1;
    int dim = (int) agent_pos.size();
    int n_pts = dim / 2;

    // 分别对x和y构建数据，为插值函数做准备
    vector<double> x, y;
    x.reserve(n_pts + 2);
    y.reserve(n_pts + 2);

    // x的数据，参考数据，以x为因变量
    x.push_back(Xs);
    for (int i = 0; i < n_pts; i++) x.push_back(agent_pos[i]);
    x.push_back(Xg);

    // y数据，因变量
    y.push_back(Ys);
    for (int i = n_pts; i < agent_pos.size(); i++) y.push_back(agent_pos[i]);
    y.push_back(Yg);

    // **************************** 插值方法 ********************************
    // 创建已有数据
    int len = n_pts + 2;
    double *x0 = new double[len];
    for (int i = 0; i < len; i++) {
        x0[i] = (1.0 - 0.0) / (len - 1) * i;
    }
    // 创建插值点
    double *ss = new double[pointNums];
    for (int i = 0; i < pointNums; i++) {
        ss[i] = (1.0 - 0.0) / (pointNums - 1) * i;
    }
    // 已有数据，和x0配合生成插值函数
    double *cur_x = new double[len];
    double *cur_y = new double[len];
    for (int i = 0; i < x.size(); i++) {
        cur_x[i] = x[i];
        cur_y[i] = y[i];
    }

    double *xx = new double[pointNums];
    double *yy = new double[pointNums];

    // Px：插值得到的x坐标  Py：插值得到的y坐标
    vector<double> Px, Py;
    Px.reserve(pointNums);
    Py.reserve(pointNums);
    try {
        SplineInterface *sp_x = new Spline(x0, cur_x, len);
        sp_x->AutoInterp(pointNums, ss, xx);
        // 将数据转化为vector类型数组
        for (int i = 0; i < pointNums; i++) {
            Px.emplace_back(xx[i]);
        }
        delete sp_x;
        sp_x = nullptr;

        SplineInterface *sp_y = new Spline(x0, cur_y, len);    //使用接口，且使用默认边界条件
        sp_y->AutoInterp(pointNums, ss, yy);            //求x的插值结果y
        // 数据转换为vector
        for (int i = 0; i < pointNums; i++) {
            Py.emplace_back(yy[i]);
        }
        delete sp_y;
        sp_y = nullptr;
    } catch (SplineFailure sf) {
        cout << sf.GetMessage() << endl;
    }

    // **************************** 替换插值方法 end ********************************

    // 构建差分数组
    vector<double> dx = numpy_diff(Px);
    vector<double> dy = numpy_diff(Py);
    double L = 0.0;
    for (int i = 0; i < dx.size(); i++) {
        L += sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
    }

    // 惩罚值
    double err;
    int count;
    tie(err, count) = path_penalty(obs, Px, Py);

    args.res = {L, count, Px, Py};

    // 释放资源
    delete[] yy;
    yy = nullptr;
    delete[] xx;
    xx = nullptr;
    delete[] cur_x;
    cur_x = nullptr;
    delete[] cur_y;
    cur_y = nullptr;
    delete[] ss;
    ss = nullptr;
    delete[] x0;
    x0 = nullptr;

    return {L, count, Px, Py};
};
//******************************** PathPlanning end ************************************





//********************************* other tools begin ***************************************
/**
 * 如果路径中的任何一点违反了任何障碍，则返回惩罚值。为了加快计算速度，算法被设计为同时处理所有点。
 * @param obs   障碍
 * @param Px    插值后的路径点x坐标
 * @param Py    插值后的路径点x坐标
 * @return
 */
tuple<double, int> path_penalty(const vector<Obstacle> &obs, vector<double> Px, vector<double> Py) {
    // 在一维参数里面，err就是一个数而已
    double err = 0.0;
    int count = 0;

    vector<bool> inside;

    for (int ii = 0; ii < obs.size(); ii++) {
        inside.clear();
        // 获取参数
        string name = obs[ii].type_name;
        double xc = obs[ii].x;
        double yc = obs[ii].y;
        double Kv = obs[ii].kv;

        // 计算距离障碍中心的距离
        vector<double> distance(Px.size());
        for (int i = 0; i < Px.size(); i++) {
            distance[i] = sqrt((Px[i] - xc) * (Px[i] - xc) + (Py[i] - yc) * (Py[i] - yc));
        }

        if (name == "Circle") {
            double r = obs[ii].real_r;
            for (double dis: distance) {
                inside.push_back(r > dis);
            }
        } else if (name == "Ellipse") {
            // 椭圆模型 暂时没有
        } else if (name == "Convex") {
            vector<vector<int>> V = obs[ii].V;
            vector<double> a(Px.size(), numeric_limits<double>::infinity());
            vector<double> side(Px.size());
            for (int i = 0; i < V.size(); i++) {
                int index = ((i - 1) + V.size()) % V.size();
                for (int j = 0; j < Px.size(); j++) {
                    side[j] = (Py[j] - V[index][1]) * (V[i][0] - V[index][0]) -
                              (Px[j] - V[index][0]) * (V[i][1] - V[index][1]);
                    a[j] = fmin(a[j], side[j]);
                }
            }
            for (double e: a) inside.push_back(e > 0.0);
        }

        // 标记是否有插值点在障碍内部，inside为true则是有
        bool flag = false;

        vector<double> penalty(Px.size(), 0.0);
        for (int j = 0; j < distance.size(); j++) {
            if (inside[j]) {
                penalty[j] = Kv / distance[j];
                flag = true;
            }
        }
        // 有插值点在障碍物内部
        if (flag) count++;

        double sum = 0.0;
        int len = 0;
        for (int j = 0; j < penalty.size(); j++) {
            sum += penalty[j];
            len++;
        }
        err += sum / len;
    }
    return {err, count};
}

/**
 *
 * @param nums      y
 * @param pointNums 插值点个数
 * @return 插值得到的数组
 */
vector<double> interpolation(const vector<double> &nums, int pointNums) {
//    // 1.初始化python解释器
//    Py_Initialize();
//    // 2.初始化python系统文件路径，保证可以访问到 .py文件
//    PyRun_SimpleString("import sys");
//    // 在CMake中，这个路径要从cmake-build-release出发或者cmake-build-debug出发
//    PyRun_SimpleString("sys.path.append('../script')");
//    // 3.调用python文件名，不用写后缀
//    PyObject *pModule = PyImport_ImportModule("interpolation_script");
//    // 4.指定调用函数
//    PyObject *pFunc = PyObject_GetAttrString(pModule, "get_interpolation_nums");
//
//    // 5.设置参数
//    // 5.1 获取一个python列表
//    PyObject *pyParams = PyList_New(0);
//    // 5.2 给列表赋值
//    for (int i = 0; i < nums.size(); i++) {
//        PyList_Append(pyParams, Py_BuildValue("d", nums[i]));
//    }
//    // 6.构造python的方法参数，两个参数
//    PyObject *args = PyTuple_New(2);
//    // 设置函数实参 变量格式转换成python格式
//    PyTuple_SetItem(args, 0, pyParams);
//    PyTuple_SetItem(args, 1, Py_BuildValue("i", pointNums));
//    // 函数调用，得到返回值列表
//    PyObject *pyList = PyEval_CallObject(pFunc, args);
//
//    int size = PyList_Size(pyList);
//    vector<double> datas;
//    datas.reserve(size);
//    PyObject *item;
//    for (int i = 0; i < size; i++) {
//        // 得到每一项
//        item = PyList_GetItem(pyList, i);
//        double d = (double) PyFloat_AsDouble(item);
//        datas.emplace_back(d);
//    }
//
//    // for (double d : datas) cout << d << ' ';
//    delete item;
//    delete pyList;
//    delete args;
//    delete pyParams;
//    delete pFunc;
//    delete pModule;
//
//    Py_Finalize();
//
//    return datas;
}

/**
 * 一维数组拷贝，类型：vector<double>
 * @param origin
 * @param target
 */
void deepcopy(const vector<double> &origin, vector<double> &target) {
    int index = 0;
    for (auto e: origin) {
        target[index++] = e;
    }
}

/**
 * 拷贝二维数组，类型：vector<vector<double>>
 * @param origin        原数组
 * @param vec_target    新数组
 */
void deepcopy(const vector<vector<double>> &origin, vector<vector<double>> &vec_target) {
    for (int i = 0; i < origin.size(); i++) {
        for (int j = 0; j < origin[i].size(); j++) {
            vec_target[i][j] = origin[i][j];
        }
    }
}


vector<vector<double>> getData() {
    vector<vector<double>> res = {
            {138.5621923, 1.70900606,  178.0007115, 168.5031066, 51.671561,   117.1022784},
            {111.9550712, 111.4858033, 63.83343806, 167.7582633, 95.43940443, 70.08856715},
            {60.601514,   70.03750618, 195.1795069, 135.1429075, 125.3400456, 151.4684157},
            {84.77397601, 41.92100561, 112.4798896, 83.94757189, 148.6859238, 130.7418553},
            {33.72658222, 155.3892724, 37.82119315, 179.9219251, 119.3106879, 118.9739923},
            {103.5261625, 140.3140634, 32.20192637, 168.6083027, 10.97369812, 88.02141517},
            {107.4543148, 12.20339379, 44.68891705, 159.0173535, 55.85452161, 181.151796},
            {18.39239031, 144.2971939, 95.81860335, 150.6868242, 122.5110143, 67.59517095},
            {125.7015682, 125.5624946, 64.4459927,  168.8303344, 130.5458194, 126.7016946},
            {136.7523894, 25.13063307, 69.56659623, 105.3169471, 1.734612834, 146.6117419},
            {197.9342189, 130.4434694, 52.40812763, 47.45732617, 99.26396565, 198.4670269},
            {109.2862082, 138.2856227, 41.28346411, 53.48563303, 16.65396302, 64.73288981},
            {63.17964398, 152.6044946, 90.1359215,  105.1180187, 121.7476468, 126.7768733},
            {141.5682694, 57.20339421, 32.50526831, 51.0811964,  27.81179256, 26.05692394},
            {72.96984586, 52.98807273, 71.63417996, 176.2681479, 155.590146,  96.61102219},
            {50.55372914, 102.0248334, 119.584925,  132.8033006, 144.8727683, 129.0415049},
            {101.7374083, 95.63974856, 37.66644974, 153.0887328, 86.79116905, 196.4118141},
            {27.69216739, 5.906870404, 98.16741608, 142.503743,  140.4026668, 164.708267},
            {149.830278,  194.7115494, 73.44764473, 92.17783245, 113.4914636, 101.5927439},
            {123.0313511, 166.4191053, 96.22120056, 55.52302052, 21.43431308, 17.66172441},
            {15.23567714, 39.32735744, 154.6323342, 145.9830168, 26.99788752, 37.66457454},
            {147.7426691, 19.37124997, 149.2390398, 134.5438456, 90.90269703, 113.2730242},
            {161.0044343, 157.2762741, 147.5960221, 42.23169812, 116.9217821, 45.90355863},
            {88.1268987,  61.75894199, 6.01471737,  54.28973052, 69.161665,   95.15043744},
            {119.4674543, 126.6833157, 39.8021711,  115.6954781, 79.8545592,  91.01222991},
            {127.6950493, 29.22782491, 76.0658297,  160.3991945, 123.7469596, 23.89924461},
            {9.79782961,  119.6383671, 141.6510409, 189.4978614, 82.49595812, 23.80081606},
            {141.6030578, 12.47056,    168.8735586, 145.0720995, 16.257492,   9.916916301},
            {174.6179575, 153.8497704, 40.81854446, 74.28032339, 173.0925106, 33.24936594},
            {51.26766163, 65.45250601, 71.48104711, 105.3575455, 48.01533444, 75.05072281},
            {98.29849897, 187.869736,  33.13808187, 123.4013427, 120.150217,  61.53296767},
            {199.8249974, 87.06384142, 180.3606785, 80.66569791, 7.39139553,  78.60586578},
            {27.15031415, 162.9906918, 122.0075848, 136.2158013, 23.96882195, 193.1060032},
            {107.5189986, 6.663181256, 100.2531465, 82.07731343, 92.04834154, 137.7863266},
            {69.71393224, 60.52699681, 152.9812739, 92.75444734, 20.74236377, 102.4315231},
            {63.13606089, 149.5570748, 105.7163903, 188.3974624, 137.4295419, 15.69720453},
            {139.81205,   185.7173657, 67.48699005, 84.43157255, 142.6964001, 197.943125},
            {136.9758493, 76.42627024, 13.8357181,  49.41047976, 17.16110007, 54.02927313},
            {24.14344482, 106.5679598, 20.73127145, 39.49491434, 142.3877027, 160.3193421},
            {18.51815381, 52.82896178, 188.897824,  54.56090681, 33.01737912, 159.0872524},
            {54.97329102, 67.43734516, 48.77553426, 199.8501506, 120.5287295, 166.2559337},
            {181.2514078, 128.8530396, 184.7726377, 58.31803739, 44.47243127, 2.275798463},
            {58.4614394,  81.08888449, 107.9429376, 188.8440215, 72.27367751, 187.4555242},
            {103.9034514, 104.6401708, 35.68392865, 189.0094335, 138.3536014, 91.39123283},
            {134.824944,  91.16352439, 75.90168703, 195.8646714, 171.6368453, 196.533907},
            {160.650255,  20.45372852, 89.15832823, 177.8591055, 150.931657,  121.6473015},
            {126.4433308, 3.106112355, 64.11046539, 168.4418334, 15.7394001,  154.6944429},
            {113.5392385, 80.02060101, 45.74858334, 152.4155449, 126.7622459, 27.93566026},
            {140.1832723, 38.12045413, 168.764462,  69.79503243, 176.0686693, 20.39235962},
            {132.239064,  84.62648068, 49.70359284, 9.343639153, 113.2302967, 185.03982},
            {157.2784303, 101.6799555, 131.2561685, 56.65811016, 114.4555084, 161.1606505},
            {64.60342223, 83.0111543,  179.8748664, 170.6687766, 19.84629177, 39.68003259},
            {101.1991462, 61.27854605, 83.48917862, 38.97109874, 58.50650085, 79.95825727},
            {66.12465695, 77.52355058, 167.6352292, 70.22820636, 154.0826552, 11.08178112},
            {140.3083291, 169.7425216, 17.50899075, 101.2335972, 125.1497453, 103.4540475},
            {23.76153887, 35.54581373, 194.9650688, 24.25590722, 148.466721,  21.33454492},
            {192.5967647, 153.5810715, 14.44496261, 143.3840649, 103.1304088, 170.3485371},
            {150.5861943, 146.6206881, 35.52965266, 30.3737834,  153.064108,  60.41311735},
            {174.1039295, 54.45002477, 122.9914813, 149.082909,  23.23014579, 65.4290187},
            {47.2941282,  23.60829394, 145.3898805, 1.945792698, 100.7886018, 37.14983873},
            {7.619712493, 105.2418411, 100.5935872, 67.84308205, 142.645847,  172.0302527},
            {49.18157559, 118.6904908, 126.1301789, 198.6993133, 4.053708689, 153.6575276},
            {94.82955802, 83.67153875, 150.1681567, 22.83228339, 34.85211306, 115.1582398},
            {171.557171,  137.3844983, 33.59944278, 180.8312313, 184.0679108, 111.6563502},
            {83.0069248,  12.16010661, 4.052396057, 65.68826885, 33.65135988, 193.0407003},
            {93.5988987,  50.17157806, 153.6998167, 90.70347993, 54.33317346, 4.610200769},
            {113.0600796, 15.72329835, 155.7985946, 93.98899482, 81.58132605, 141.3336626},
            {14.46218737, 128.243196,  174.2244907, 41.03373866, 148.067426,  177.0380185},
            {160.7822671, 1.238406405, 91.4684007,  166.3290086, 30.1370353,  92.17651456},
            {187.3304169, 87.76316393, 69.18770706, 94.44821329, 113.402426,  144.5096353},
            {182.0421241, 75.53132987, 196.4440786, 8.860004155, 32.17897687, 97.15728775},
            {6.930003693, 132.6901353, 92.84594667, 138.5286031, 19.09287664, 165.2869202},
            {120.0405053, 90.93378013, 159.5514657, 29.35299639, 125.6428674, 10.16875813},
            {155.0187517, 167.1463263, 187.7385193, 59.79167311, 0.966598511, 166.676826},
            {67.8632722,  140.6680852, 90.98284895, 23.52780991, 182.5690158, 171.4140605},
            {162.1412875, 124.7725828, 106.9154319, 134.9436937, 126.8869557, 23.76083448},
            {14.06392543, 172.5681936, 116.8772932, 17.59687545, 112.0989054, 105.8989951},
            {24.9131988,  58.71178596, 199.4512396, 113.8111863, 5.113139341, 81.80046281},
            {135.0022761, 149.2136321, 35.1411923,  47.69615751, 99.0795584,  125.8760604},
            {94.37724794, 156.5761423, 23.38804125, 86.45145327, 147.0302111, 173.1474825},
            {71.84943867, 76.44722321, 43.81969233, 85.97406065, 35.31990547, 67.89315035},
            {4.585783834, 46.76818564, 4.874422185, 91.04366865, 101.0677404, 188.5665088},
            {124.1896554, 89.77409869, 159.959885,  119.3078385, 107.2213589, 126.1418675},
            {50.71644618, 20.42757224, 83.47032676, 119.2954904, 78.10501773, 13.90717373},
            {176.5390494, 72.87212981, 190.7639119, 94.4801991,  185.6493467, 96.23618451},
            {169.5944846, 2.972239379, 153.152192,  3.074104899, 7.543569437, 74.27054632},
            {97.32240003, 29.61533064, 48.69791424, 118.6700695, 17.68654031, 62.19946855},
            {96.70232388, 121.5203854, 11.72157156, 153.4368936, 78.81656147, 169.5898457},
            {93.02943391, 4.985461741, 77.70633343, 86.3076302,  141.159664,  74.44300435},
            {155.9688287, 159.4384514, 12.00610834, 156.073845,  10.05264467, 184.267289},
            {193.1332598, 193.8609172, 7.136526697, 111.789205,  178.9003363, 27.36453597},
            {134.5617831, 182.837653,  15.0671394,  139.175653,  151.2934598, 129.7431373},
            {82.75967486, 139.3994206, 41.74465764, 156.4763259, 18.7006665,  192.6645517},
            {167.4860646, 0.5975545,   52.79308248, 195.5028186, 79.96739199, 96.02407592},
            {26.75202472, 149.4044872, 81.37209434, 45.14292599, 52.98646731, 41.68408359},
            {46.38745375, 114.5625473, 44.14839453, 128.1295971, 117.1114254, 93.94271631},
            {108.9208579, 161.6278667, 112.339614,  67.68150351, 153.5652324, 8.165959326},
            {95.97407309, 52.60679915, 102.6298465, 174.9156428, 94.51372879, 118.5735609},
            {125.5482962, 98.53869025, 35.03275653, 40.25170072, 69.66476648, 75.27280383},
            {156.4648864, 144.3674568, 57.56347993, 113.2533761, 186.2529118, 34.88779264}
    };
    return res;
}

//********************************* other tools end ************************************************