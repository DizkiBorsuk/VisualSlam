cmake_minimum_required(VERSION 3.15)
project(visual_slam_cpp VERSION 0.1.0 LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)

set(CMAKE_TOOLCHAIN_FILE "C:/Users/Maciek/Desktop/dev_workspace/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(CMAKE_PREFIX_PATH "C:/Users/Maciek/Desktop/dev_workspace/vcpkg/installed/x64-windows/share")

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3  REQUIRED NO_MODULE)
find_package(g2o CONFIG REQUIRED)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)

include(FetchContent)
FetchContent_Declare(
    matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
)

FetchContent_GetProperties(matplotlib)
if(NOT matplotlib_POPULATED)
    FetchContent_Populate(matplotlib)
endif()
include_directories( ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${matplotlib_SOURCE_DIR})
#include_directories(SYSTEM ${matplotlib_SOURCE_DIR})


add_executable(${PROJECT_NAME} src/main.cpp src/visualize_data.cpp src/readDataset.cpp src/SLAM.cpp src/FeatureExtraction.cpp)
target_link_libraries(${PROJECT_NAME} 
                      ${OpenCV_LIBS} 
                      Eigen3::Eigen 
                      ${Pangolin_LIBRARIES}
                      g2o::core g2o::stuff g2o::types_data g2o::types_icp g2o::types_slam3d g2o::types_slam2d
                      ${PYTHON_LIBRARIES}
                      Python3::NumPy
                      )

