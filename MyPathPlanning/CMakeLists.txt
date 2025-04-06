cmake_minimum_required(VERSION 3.27)
project(MyPathPlanning)

set(CMAKE_CXX_STANDARD 17)


##################################### python 环境配置 ########################################
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})


add_executable(MyPathPlanning main.cpp
        Hungarian.cpp
        Hungarian.h
        PathPlanning.cpp
        PathPlanning.h
        data.h
        data.cpp
        run_example.h
        run_example.cpp
        utils.h
        utils.cpp
        obstacle.h
        args.h
        result.h
        Spline.h
        Spline.cpp
        PSOAFSA.cpp
        PSOAFSA.h
        conf.h
        Example.cpp
        Example.h
)

target_link_libraries(MyPathPlanning ${Python3_LIBRARIES})
#include_directories("D:/soft/Python/Python37/Lib/site-packages/numpy/core/include")
