#include "utils.h"

/**
 * 计算cost，以觅食、聚群、追尾三种方式，返回三种方式中最好的pos和cost
 * 我也不懂这个函数名字为什么这样取，只能说源码确实挺像屎的
 * @param agent_pos
 * @param args
 * @return
 */
tuple<vector<double>, vector<vector<double>>, vector<double>>
path_length(const vector<vector<double>> &agent_pos, const Args &args) {

    // 不进行行为下的cost
    vector<double> f = calc_path_length(agent_pos, args);

    // 觅食行为下找到的更好的随机点以及此随机点下的cost
    vector<vector<double>> forage_agent_pos;
    vector<double> forage_cost;
    tie(forage_agent_pos, forage_cost) = forage(agent_pos, f, args, 20, 5);

    // 聚群行为下找到的更好的点以及此点下的cost
    vector<vector<double>> huddle_agent_pos;
    vector<double> huddle_cost;
    tie(huddle_agent_pos, huddle_cost) = huddle(agent_pos, args, forage_agent_pos, forage_cost);

    // 追尾行为下找到的更好的点以及此点下的cost
    vector<vector<double>> follow_agent_pos;
    vector<double> follow_cost;
    tie(follow_agent_pos, follow_cost) = follow(agent_pos, args, huddle_agent_pos, huddle_cost, f);

    return {f, follow_agent_pos, follow_cost};
}

/**
 * 返回最小化的函数，即当没有任何障碍冲突时的路径长度。
 * calc_path_length for vector<vector<double>>
 * @param agent_pos
 * @param args
 * @return
 */
vector<double> calc_path_length(vector<vector<double>> agent_pos, const Args &args) {
    int len = (int) args.starts_tdv.size();
    vector<double> Xs, Ys, Xg, Yg;
    Xs.reserve(len);
    Ys.reserve(len);
    Xg.reserve(len);
    Yg.reserve(len);

    for (int i = 0; i < len; i++) {
        Xs.push_back(args.starts_tdv[i][0]);
        Ys.push_back(args.starts_tdv[i][1]);
    }

    for (int i = 0; i < len; i++) {
        Xg.push_back(args.goals_tdv[i][0]);
        Yg.push_back(args.goals_tdv[i][1]);
    }

    vector<Obstacle> obs = args.obs;
    int pointNums = args.interpolationPointNums;
    // 插值方式，其实完全可以不用，因为使用的默认就是cubic方式
    string f_interp_type = args.interpolationKind;

    // n_pop：粒子数量 dim：粒子的维度
    int n_pop = (int) agent_pos.size();
    int dim = (int) agent_pos[0].size();
    // 用来切数据用的，将n_var的维度切割两半
    int n_pts = dim / 2;

    // 断点的坐标(开始+内部+目标)
    vector<vector<double>> x, y;
    x.reserve(n_pop);
    y.reserve(n_pop);

    vector<double> row;
    for (int i = 0; i < n_pop; i++) {
        row.clear();
        row.reserve(n_pts + 2);

        row.push_back(Xs[i]);
        for (int j = 0; j < n_pts; j++) {
            row.push_back(agent_pos[i][j]);
        }
        row.push_back(Xg[i]);

        x.push_back(row);
    }

    for (int i = 0; i < n_pop; i++) {
        row.clear();
        row.reserve(n_pts + 2);

        row.push_back(Ys[i]);
        for (int j = n_pts; j < dim; j++) {
            row.push_back(agent_pos[i][j]);
        }
        row.push_back(Yg[i]);

        y.push_back(row);
    }

    // ************************* 插值 *******************************
    /*
     * 下面进入插值
     * Px, Py 分别表示插值后坐标点，其中：
     * (1) Px：表示插值的x坐标
     * (2) Py：表示插值的y坐标
     * */
    vector<vector<double>> Px, Py;
    Px.reserve(n_pop);
    Py.reserve(n_pop);

    // 插值需要先使用数据拟合一个函数
    // 这里使用t数组作为拟合的x坐标
    int interpolation_length = n_pts + 2;
    double *t = new double[interpolation_length];
    for (int i = 0; i < interpolation_length; i++) {
        t[i] = (1.0 - 0.0) / (interpolation_length - 1) * i;
    }

    // 使用s数组作为带插值点的自变量x坐标，用其去求y坐标
    double *s = new double[pointNums];
    for (int i = 0; i < pointNums; i++) {
        s[i] = (1.0 - 0.0) / (pointNums - 1) * i;
    }

    // cur_x，cur_y存放当前拟合函数的y坐标，和cur_y配合生成插值函数
    double *cur_x = new double[interpolation_length];
    double *cur_y = new double[interpolation_length];
    double *yy = new double[pointNums];
    // 一行一行的进行插值
    for (int i = 0; i < n_pop; i++) {
        // 获取每一行的值，然后插值后再换下一行
        for (int j = 0; j < interpolation_length; j++) {
            cur_x[j] = x[i][j];
            cur_y[j] = y[i][j];
        }
        try {
            // 使用t为自变量，cur_x为因变量拟合数据
            SplineInterface *sp_x = new Spline(t, cur_x, interpolation_length);
            // 得到x的拟合数据（一行）
            sp_x->AutoInterp(pointNums, s, yy);            //求x的插值结果y
            vector<double> cur_row_x(yy, yy + pointNums);
            Px.push_back(cur_row_x);
            delete sp_x;

            // 同理，插值y数据一行
            SplineInterface *sp_y = new Spline(t, cur_y, interpolation_length);
            sp_y->AutoInterp(pointNums, s, yy);
            vector<double> cur_row_y(yy, yy + pointNums);
            Py.push_back(cur_row_y);
            delete sp_y;
        } catch (SplineFailure sf) {
            cout << sf.GetMessage() << endl;
        }
    }
    // 插值完成后释放指针
    delete[] yy;
    delete[] cur_y;
    delete[] cur_x;
    // ************************* 插值 end *******************************


    /*
     * 计算路径长度
     * (1)求差分：Px和Py分别求差分
     * (2)使用差分数组求整条线的距离
     * */
    // 求差分数组
    vector<vector<double>> dx = numpy_diff(Px);
    vector<vector<double>> dy = numpy_diff(Py);

    vector<double> L;
    L.reserve(n_pop);
    for (int i = 0; i < dx.size(); i++) {
        double sum = 0.0;
        for (int j = 0; j < dx[i].size(); j++) {
            sum += sqrt(dx[i][j] * dx[i][j] + dy[i][j] * dy[i][j]);
        }
        L.push_back(sum);
    }

    // 惩罚值 用来修正路线 但某个点在路径中时 用此方法来修正
    vector<double> err;
    int count;
    tie(err, count) = path_penalty(obs, Px, Py);
    // 本次损失
    vector<double> cur_f;
    for (int i = 0; i < n_pop; i++) {
//        cur_f.push_back(L[i] * (1.0 + err[i]));
        cur_f.push_back(L[i] * (0.4 + err[i]));
    }

    // 释放内存
    delete[] t;
    delete[] s;

    return cur_f;
}


/**
 * 如果路径中的任何一点违反了任何障碍，则返回惩罚值。为了加快计算速度，算法被设计为同时处理所有点。
 * @param obs  障碍
 * @param Px    插值后的路径点x坐标
 * @param Py    插值后的路径点y坐标
 * @return
 */
tuple<vector<double>, int>
path_penalty(const vector<Obstacle> &obs, vector<vector<double>> Px, vector<vector<double>> Py) {
    // 粒子数 维度
    int n_pop = (int) Px.size();
    int dim = (int) Px[0].size();

    // 返回值：err 惩戒值
    vector<double> err(n_pop);
    // 在障碍中的点
    int count = 0;

    // 标记数组，标记某个点是否在障碍内
    vector<vector<bool>> inside(n_pop, vector<bool>(dim));

    for (int ii = 0; ii < obs.size(); ii++) {
        string name = obs[ii].type_name;
        double xc = obs[ii].x;
        double yc = obs[ii].y;
        double r;
        double Kv = obs[ii].kv;

        // 距离 插值点与中心点的距离 用于判断圆形区域
        vector<vector<double>> distance(n_pop, vector<double>(dim));
        for (int i = 0; i < n_pop; i++) {
            for (int j = 0; j < dim; j++) {
                distance[i][j] = sqrt((Px[i][j] - xc) * (Px[i][j] - xc) + (Py[i][j] - yc) * (Py[i][j] - yc));
            }
        }
        if (name == "Circle") {
            r = obs[ii].real_r;
            // 计算距离，判断位置
            for (int i = 0; i < n_pop; i++) {
                for (int j = 0; j < dim; j++) {
                    // 判断插值点是否在障碍内部
                    inside[i][j] = r > distance[i][j];
                }
            }
        } else if (name == "Ellipse") {
            // Ellipse
        } else if (name == "Convex") {
            // 获取不规则障碍的点坐标
            vector<vector<int>> V = obs[ii].V;

            vector<vector<double>> a(n_pop, vector<double>(dim, numeric_limits<double>::infinity()));
            for (int i = 0; i < V.size(); i++) {
                int index = ((i - 1) + V.size()) % V.size();
                for (int m = 0; m < n_pop; m++) {
                    for (int n = 0; n < dim; n++) {
                        double side = (Py[m][n] - V[index][1]) * (V[i][0] - V[index][0]) -
                                     (Px[m][n] - V[index][0]) * (V[i][1] - V[index][1]);
                        a[m][n] = fmin(a[m][n], side);
                    }
                }
            }

            for (int i = 0; i < n_pop; i++) {
                for (int j = 0; j < dim; j++) {
                    inside[i][j] = a[i][j] > 0.0;
                }
            }
        }

        // 标记，标记 inside 中是否有 true
        bool flag = false;
        vector<vector<double>> penalty(n_pop, vector<double>(dim, 0.0));
        for (int j = 0; j < n_pop; j++) {
            for (int k = 0; k < dim; k++) {
                if (inside[j][k]) {
                    penalty[j][k] = (double) Kv / distance[j][k];
                    flag = true;
                }
            }
        }
        if (flag) count++;
        for (int i = 0; i < n_pop; i++) {
            double sum = 0.0;
            for (int j = 0; j < dim; j++) {
                sum += penalty[i][j];
            }
            err[i] += sum / dim;
        }
    }
    return {err, count};
}

/**
 * 觅食行为，粒子再视野范围内找一个随机点，计算这个随机点的cost
 * @param agent_pos     粒子位置
 * @param agent_cost    粒子当前位置的cost
 * @param args          参数
 * @param Visual        视野
 * @param trynum        寻找次数
 * @return              返回在尝试次数内每个粒子找到的更好的随机点坐标以及cost，没有找到的就返回自身
 */
tuple<vector<vector<double>>, vector<double>>
forage(const vector<vector<double>> &agent_pos, const vector<double> &agent_cost, const Args &args, int Visual,
       int trynum) {

    vector<double> forage_cost(agent_cost);
    vector<vector<double>> forage_agent_pos(agent_pos);

    for (int num = 0; num < trynum; num ++) {
        // 使用随机移动策略
        vector<vector<double>> new_agent_pos = moveRandomly(agent_pos, Visual);
        vector<double> new_cost = calc_path_length(new_agent_pos, args);
        for (int i = 0; i < new_cost.size(); i++) {
            // 如果找到更好的位置
            if (new_cost[i] < forage_cost[i]) {
                forage_agent_pos[i].assign(new_agent_pos[i].begin(), new_agent_pos[i].end());
                forage_cost[i] = new_cost[i];
            }
        }
    }
    return {forage_agent_pos, forage_cost};
}

/**
 * 聚群行为  粒子在其视野范围种群最优中心位置，若该位置不拥挤，且cost更优，粒子则根据该位置和全局最优粒子更新速度
 * @param agent_pos         粒子位置
 * @param args              参数
 * @param forage_agent_pos  觅食行为下的位置
 * @param forage_cost       觅食行为下的cost
 * @return
 */
tuple<vector<vector<double>>, vector<double>>
huddle(vector<vector<double>> agent_pos, const Args &args, const vector<vector<double>> &forage_agent_pos,
       vector<double> forage_cost) {

    int n_pop = (int) agent_pos.size();
    int dim = (int) agent_pos[0].size();

    vector<double> huddle_cost(forage_cost);
    vector<vector<double>> huddle_agent_pos(forage_agent_pos);

    // 求中心位置
    vector<vector<double>> center_agent_pos(n_pop, vector<double>(dim));
    // 存储求得平均数
    vector<double> avg;
    for (int i = 0; i < n_pop; i++) {
        // 首先，删除行，删除第i行
        vector<vector<double>> agent_pos_copy(agent_pos);
        agent_pos_copy.erase(agent_pos_copy.begin() + i);

        avg.clear();
        avg.reserve(dim);
        // 按列求平均数
        for (int k = 0; k < agent_pos_copy[0].size(); k++) {
            double col_sum = 0.0;
            for (int j = 0; j < agent_pos_copy.size(); j++) {
                col_sum += agent_pos_copy[j][k];
            }
            avg.push_back(col_sum / (n_pop - 1));
        }
        center_agent_pos[i].assign(avg.begin(), avg.end());
    }

    // 计算中心位置的 cost
    vector<double> new_F = calc_path_length(center_agent_pos, args);

    for (int i = 0; i < forage_cost.size(); i++) {
        if (new_F[i] < forage_cost[i]) {
            huddle_agent_pos[i].assign(center_agent_pos[i].begin(), center_agent_pos[i].end());
            huddle_cost[i] = new_F[i];
        }

    }
    return {huddle_agent_pos, huddle_cost};
}

/**
 * 追尾行为  粒子在其视野范围内找最优粒子，若最优粒子的范围不拥挤，粒子则根据范围内的最优粒子和全局最优粒子更新速度
 * @param agent_pos         粒子位置
 * @param args              参数
 * @param huddle_agent_pos  觅食和聚群行为下的位置
 * @param huddle_cost       觅食和聚群行为下的cost
 * @param F                 无行为下的cost
 * @param Visual
 * @return
 */
tuple<vector<vector<double>>, vector<double>>
follow(const vector<vector<double>> &agent_pos, const Args &args, vector<vector<double>> huddle_agent_pos,
       const vector<double> &huddle_cost, const vector<double> &f, int Visual) {

    vector<double> follow_cost(huddle_cost);
    vector<vector<double>> follow_agent_pos(huddle_agent_pos);

    // 寻找视野内的最优粒子
    for (int i = 0; i < follow_cost.size() - 1; i++) {
        if (follow_cost[i] > follow_cost[i + 1]) {
            follow_cost[i] = follow_cost[i + 1];
            follow_agent_pos[i] = follow_agent_pos[i + 1];
        }
    }
    return {follow_agent_pos, follow_cost};
}

/**
 * 粒子随机移动策略
 * @param agent_pos
 * @param Visual
 * @return
 */
vector<vector<double>> moveRandomly(const vector<vector<double>> &agent_pos, double Visual) {

    vector<vector<double>> new_agent_pos = agent_pos;

    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);

    for (int i = 0; i < agent_pos.size(); i++) {
        for (int j = 0; j < agent_pos[i].size(); j++) {
            double e = Visual * (2 * dis(gen) - 1);
            new_agent_pos[i][j] += e;
        }
    }
    return new_agent_pos;
}


/**
 * 对出界的粒子使用速度反向限制
 * 对每一位做一个随机数相乘
 * @param agent_vel
 * @return
 */
vector<vector<double>> random_back_conf(const vector<vector<double>> &agent_vel) {
    int n_pop = (int) agent_vel.size();
    int dim = (int) agent_vel[0].size();

    // c++11中引入的随机数
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);

    // 矩阵大小相同，相当于预置矩阵的大小
    vector<vector<double>> vel_conf(n_pop, vector<double>(dim));
    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < dim; j++) {
            vel_conf[i][j] = -dis(gen) * agent_vel[i][j];
        }
    }
    // 返回处理结果
    return vel_conf;
}

/**
 * 对出界粒子速度应用双曲约束
 * @param agent_pos 位置
 * @param agent_vel 速度
 * @param UB    上限
 * @param LB    下限
 * @return
 */
vector<vector<double>>
hyperbolic_conf(vector<vector<double>> &agent_pos, vector<vector<double>> &agent_vel, vector<int> UB, vector<int> LB) {

    int n_pop = agent_vel.size();
    int n_var = agent_vel[0].size();

    vector<vector<double>> vel_pos(n_pop, vector<double>(n_var));
    vector<vector<double>> vel_neg(n_pop, vector<double>(n_var));

    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < n_var; j++) {
            vel_pos[i][j] = agent_vel[i][j] / (1.0 + abs(agent_vel[i][j] / (UB[j] - agent_pos[i][j])));
            vel_neg[i][j] = agent_vel[i][j] / (1.0 + abs(agent_vel[i][j] / (agent_pos[i][j] - LB[j])));
        }
    }

    vector<vector<double>> vel_conf(n_pop, vector<double>(n_var));

    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < n_var; j++) {
            vel_conf[i][j] = agent_vel[i][j] > 0 ? vel_pos[i][j] : vel_neg[i][j];
        }
    }
    return vel_conf;
}

/**
 * 对出界粒子速度进行混合约束（随机选择）
 * @param agent_pos
 * @param agent_vel
 * @param UB
 * @param LB
 * @return
 */
vector<vector<double>>
mixed_conf(vector<vector<double>> &agent_pos, vector<vector<double>> &agent_vel, vector<int> UB, vector<int> LB) {

    int n_pop = agent_vel.size();
    int n_var = agent_vel[0].size();

    // 双曲约束
    vector<vector<double>> vel_hy = hyperbolic_conf(agent_pos, agent_vel, UB, LB);
    // 方向约束
    vector<vector<double>> vel_rb = random_back_conf(agent_vel);

    vector<vector<double>> vel_conf(n_pop, vector<double>(n_var));

    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);

    for (int i = 0; i < n_pop; i++) {
        for (int j = 0; j < n_var; j++) {
            vel_conf[i][j] = dis(gen) >= 0.5 ? vel_hy[i][j] : vel_rb[i][j];
        }
    }

    return vel_conf;
}


void printCost(const vector<double> &cost) {
    cout << "[" << cost[0];
    for (int i = 1; i < cost.size(); i++) {
        cout << ", " << cost[i];
    }
    cout << "]" << endl;
}