#include <cstdint>
#include "PathPlanning.h"

/**
 * 无参构造器
 */
PathPlanning::PathPlanning() {};

/**
 * 有参构造
 * @param limits 空间限制
 */
PathPlanning::PathPlanning(const vector<int> &limits) {
    this->limits = limits;
}

/**
 * 有参构造器
 * @param start
 * @param goal
 * @param limits
 */
PathPlanning::PathPlanning(vector<int> start, vector<int> goal, vector<int> limits) {
    // 提取(x, y)
    this->start.assign(start.begin(), start.begin() + 2);
    // 所有数据都提取
    this->goal.assign(goal.begin(), goal.begin() + 12);
    // 和goal数据一致
    this->target = goal;
    // 空间限制全部都要
    this->limits.assign(limits.begin(), limits.begin() + 5);
}

/**
 * 重载运算符，输出类本身信息
 * @param os
 * @param pp
 * @return
 */
ostream &operator<<(ostream &os, const PathPlanning &pp) {
    os << "\nPathPlanning object"
       << "\n- start = [" << pp.start[0] << ", " << pp.start[1] << "]"
       << "\n- goal = [" << pp.goal[0] << ", " << pp.goal[1]
       << "\n- limits = " << pp.limits[0] << ", " << pp.limits[1] << ", " << pp.limits[2] << ", " << pp.limits[3]
       << "\n- number of obstacles = " << pp.obs.size();
    return os;
}

/**
 * 障碍信息
 */
void PathPlanning::obs_info() {

    size_t obs_count = obs.size();

    if (obs_count > 0) {
        cout << "<------------------------Obstacles-------------------------->\n";
    } else {
        cout << "<-----------------------No Obstacles------------------------>\n";
    }

    for (size_t i = 0; i < obs_count; ++i) {
        auto &data = obs[i];
        /*
         * type_name：障碍类型
         * 中心点：(x, y)
         * */
        string type_name;
        double x, y, r, Kv;

        /*
         * 首先，赋值无区别参数：类型、坐标(x, y)
         * */
        type_name = data.type_name;
        x = data.x;
        y = data.y;
        r = data.real_r;
        Kv = data.kv;
        // 圆类型
        if (type_name == "Circle") {
            cout << "\n" << type_name
                 << "\n- centroid = (" << x << ", " << y << ")"
                 << "\n- radius = " << r
                 << "\n- scaling factor = " << Kv << "\n";
        } else if (type_name == "Ellipse") { // 椭圆类型
            double theta, b, e;
            // get函数，获取元组的数据引用，get<3>(data): 获取data的第3个元素
//            theta = get<3>(data) * 180.0 / M_PI;
//            b = get<4>(data);
//            e = get<5>(data);
//            Kv = get<6>(data);

            double a = b / sqrt(1.0 - e * e);
            std::cout << "\n" << type_name
                      << "\n- centroid = (" << x << ", " << y << ")"
                      << "\n- rotation from x-axis= " << theta
                      << "\n- semi-major axis = " << a
                      << "\n- semi-minor axis = " << b
                      << "\n- scaling factor = " << Kv << "\n";
        }
    }

}

/**
 * 添加圆形障碍（其实这里是将type_name更改，或许有利于绘图）
 * @param x
 * @param y
 * @param r     切面圆半径
 * @param Kv    比例尺
 */
void PathPlanning::add_circle_obs(double x, double y, double r, double Kv) {
    this->obs.push_back({"Circle", x, y, r, Kv});
}

/**
 * 添加椭圆切面
 * @param x
 * @param y
 * @param theta
 * @param a
 * @param b
 * @param Kv
 */
void PathPlanning::add_ellipse_obs(double x, double y, double theta, double a, double b, double Kv) {
    double e = sqrt(1.0 - (b * b) / (a * a));
    // obs.emplace_back("Ellipse", x, y, theta, b, e, Kv);
}

/**
 * 添加不规则图形障碍
 * @param x
 * @param y
 * @param V     各个端点的坐标
 * @param Kv
 */
void PathPlanning::add_convex(double x, double y, vector<vector<int>> V, double Kv) {
    obs.push_back({"Convex", x, y, 0.0, Kv, V});
}

/**
 * 根据 id 删除障碍
 * @param idx
 */
void PathPlanning::remove_obs(size_t idx) {
    if (idx < obs.size()) {
        obs.erase(obs.begin() + idx);
        cout << "删除成功\n";
    } else {
        cout << "没有该障碍\n";
    }
}


/**
 * 一维数组差分
 * @param origin    原数组
 * @return          差分数组
 */
vector<double> PathPlanning::diff(const vector<double> &origin) {
    vector<double> res;
    // 预分配空间，增加效率
    res.reserve(origin.size() - 1);
    for (int i = 0; i < origin.size() - 1; i++) {
        res.emplace_back(origin[i + 1] - origin[i]);
    }
    return res;
}

/**
 * 如果路径中的任何一点违反了任何障碍，则返回惩罚值。为了加快计算速度，算法被设计为同时处理所有点。
 * @param obs   障碍
 * @param Px    插值后的路径点x坐标
 * @param Py    插值后的路径点x坐标
 * @return
 */
tuple<double, int> PathPlanning::path_penalty(const vector<Obstacle> &obs, vector<double> Px, vector<double> Py) {
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
 * 返回最小化的函数，即当没有任何障碍冲突时的路径长度。
 * PathPlanning USE
 * calc_path_length for vector<double>
 * @param agent_pos
 * @param args
 * @return
 */
tuple<double, int, vector<double>, vector<double>>
PathPlanning::calc_path_length(vector<double> agent_pos, Args &args) {
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
    vector<double> dx = diff(Px);
    vector<double> dy = diff(Py);
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



 /**
  *
  * @param n_pts 粒子包含的点个数，也就是维度/2
  * @param pointNums 计算插值点个数
  * @param n_pop 粒子数量
  * @param epochs 迭代次数
  * @param phi 计算权重
  * @param vel_fact 计算最大最小速度
  * @param rad 判断粒子是否符合条件
  * @param interpolation_mode 插值方法
  * @param conf_type 约束类型
  * @return 断点坐标  总路径长度  路径中有几个点在障碍物内  插值的x坐标  插值的y坐标
  */
tuple<vector<double>, double, int, vector<double>, vector<double>>
PathPlanning::optimize(int n_pts, int pointNums, int n_pop, int epochs, double phi, double vel_fact, double rad,
                       const string& interpolation_mode, const string& conf_type) {
    // 指定空间边界
    int nVar = 2 * n_pts;
    vector<int> LB(nVar, 0);
    vector<int> UB(nVar, 0);
    for (int i = 0; i < nVar; i++) {
        if (i < n_pts) {
            LB[i] = this->limits[0];
            UB[i] = this->limits[1];
        } else {
            LB[i] = this->limits[2];
            UB[i] = this->limits[3];
        }
    }

    // 构造参数
    // 其实位置
    vector<int> start_vec;
    start_vec.push_back(start[0]);
    start_vec.push_back(start[1]);
    vector<vector<int>> ss(n_pop, start_vec);
    // 目标位置
    vector<int> goal_vec;
    goal_vec.push_back(goal[0]);
    goal_vec.push_back(goal[1]);
    vector<vector<int>> gg(n_pop, goal_vec);

    Args args(ss, gg, this->obs, pointNums, interpolation_mode);

    /*
     * 参数声明：
     * best_pos：种群最优值
     * info：{最优cost，最优粒子索引，小于rad的粒子数}
     * */
    vector<double> best_pos;
    tuple<double, int, int> info;

    // 使用PSO算法优化 得到最优路径{内部断点，info}
    // tie(best_pos, info) = PSO(LB, UB, n_pop, epochs, phi, vel_fact, rad, args ,conf_type, f_interp);
    PSOAFSA pso_afsa(seed);
     pso_afsa.setVisual(this->visual);
    tie(best_pos, info) = pso_afsa.PSO_AFSA(LB, UB, n_pop, epochs, phi, vel_fact, rad, args ,conf_type, f_interp);

    /*
     * 参数声明：
     * L: 总路径长度
     * count: 路径中有几个点在障碍物内
     * Px: 插值的x坐标
     * Py: 插值的y坐标
     * */
    double L;
    int count;
    vector<double> Px, Py;
    Args args2(this->start, this->goal, this->obs, pointNums, interpolation_mode);
    tie(L, count, Px, Py) = calc_path_length(best_pos, args2);

    // 更新结果
    this->res = {best_pos, L, count, Px, Py};

    return {best_pos, L, count, Px, Py};
}

// ********************************* getter and setter ***********************************
const vector<int> &PathPlanning::getStart() const {
    return start;
}

void PathPlanning::setStart(const vector<int> &start) {
    PathPlanning::start = start;
}

const vector<int> &PathPlanning::getGoal() const {
    return goal;
}

void PathPlanning::setGoal(const vector<int> &goal) {
    PathPlanning::goal = goal;
}

const vector<int> &PathPlanning::getTarget() const {
    return target;
}

void PathPlanning::setTarget(const vector<int> &target) {
    PathPlanning::target = target;
}

const vector<int> &PathPlanning::getLimits() const {
    return limits;
}

void PathPlanning::setLimits(const vector<int> &limits) {
    PathPlanning::limits = limits;
}

const vector<Obstacle> &PathPlanning::getObs() const {
    return obs;
}

void PathPlanning::setObs(const vector<Obstacle> &obs) {
    PathPlanning::obs = obs;
}

const Result &PathPlanning::getRes() const {
    return res;
}

void PathPlanning::setRes(const Result &res) {
    PathPlanning::res = res;
}

uint32_t PathPlanning::getSeed() const {
    return seed;
}

void PathPlanning::setSeed(uint32_t seed) {
    PathPlanning::seed = seed;
}

int PathPlanning::getVisual() const {
    return visual;
}

void PathPlanning::setVisual(int visual) {
    PathPlanning::visual = visual;
}








