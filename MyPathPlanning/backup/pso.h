#ifndef MYPATHPLANNING_PSO_H
#define MYPATHPLANNING_PSO_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <ctime>
#include <random>
#include <algorithm>
#include <limits>

#include "../args.h"
#include "../utils.h"

using namespace std;

// PSO 算法
tuple<vector<double>, tuple<double, int, int>>
PSO(vector<int> LB, vector<int> UB, int n_pop, int epochs, double phi, double vel_fact, double rad, const Args &args, const string &conf_type, string interpolation_mode);
#endif //MYPATHPLANNING_PSO_H
