#ifndef MYPATHPLANNING_RUN_EXAMPLE_H
#define MYPATHPLANNING_RUN_EXAMPLE_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <vector>

#include "data.h"
#include "utils.h"
#include "result.h"
#include "PathPlanning.h"

using namespace std;

void run_example_1();

tuple<vector<vector<double>>, vector<Result>> run_example();

#endif //MYPATHPLANNING_RUN_EXAMPLE_H
