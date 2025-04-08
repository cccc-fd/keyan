cmake_minimum_required(VERSION 3.27)
project(MyPathPlanning)

set(CMAKE_CXX_STANDARD 17)

###################### Boost库配置 ######################
#set(BOOST_ROOT "D:/soft/boost_1_80_0")
#set(BOOST_ROOT "D:/soft/boost_1_78_0")
#set(Boost_ARCHITECTURE "-x64")
#find_package(Boost REQUIRED)    # 查找Boost库
#include_directories(${Boost_INCLUDE_DIRS})    # 包含Boost库的头文件目录

###################### Eigen3库配置 ######################
#find_package (Eigen3 3.3 REQUIRED NO_MODULE)

###################### 插值库配置 ######################
#find_package(libInterpolate REQUIRED)

###################### matplotlib-cpp库配置 ######################
#include_directories("D:/soft/matplotlib-cpp")
#find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
#include_directories(${Python3_INCLUDE_DIRS})
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

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
        pso.cpp
        pso_parallel.cpp
        obstacle.h
        args.h
        pso.h
        result.h
        utils_python.h
        utils_python.cpp
        utils_pso.cpp
        utils_plot.cpp
        Spline.h
        Spline.cpp
)

#target_link_libraries (MyPathPlanning Eigen3::Eigen)
#target_link_libraries(MyPathPlanning ${Boost_LIBRARIES})    # 链接Boost库
#target_link_libraries(MyPathPlanning libInterpolate::Interpolate )

#target_link_libraries(MyPathPlanning ${Python3_LIBRARIES})
#include_directories("D:/soft/Python/Python37/Lib/site-packages/numpy/core/include")

# 链接 OpenMP 库
target_link_libraries(MyPathPlanning PRIVATE OpenMP::OpenMP_CXX)