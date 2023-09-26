cmake_minimum_required(VERSION 3.25)
project(visual_slam_cpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

set(GFLAGS_USE_TARGET_NAMESPACE ON)


IF(WIN32)
    set(CMAKE_TOOLCHAIN_FILE "C:/Users/Maciek/Desktop/dev_workspace/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_PREFIX_PATH "C:/Users/Maciek/Desktop/dev_workspace/vcpkg/installed/x64-windows/share")
ELSE()
    set(CMAKE_CXX_FLAGS "-Wall -Wextra")
    set(CMAKE_CXX_FLAGS_DEBUG "-g")
    set(CMAKE_CXX_FLAGS_RELEASE "-Ofast") 
ENDIF()


find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(Eigen3  REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(g2o CONFIG REQUIRED)
find_package(Sophus CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(gflags CONFIG)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3 REQUIRED)

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

set(Sources src/main.cpp 
            src/backend_optimization.cpp
            src/camera.cpp
            src/frame.cpp
            src/map.cpp
            src/read_dataset.cpp
            src/SLAM.cpp
            src/tools.cpp
            src/tracking.cpp
            src/visualizer.cpp
            )

set(Headers include/camera.hpp 
            include/backend_optimization.hpp 
            include/common_includes.hpp
            include/frame.hpp 
            include/map.hpp
            include/read_dataset.hpp
            include/SLAM.hpp 
            include/tools.hpp
            include/tracking.hpp
            include/visualizer.hpp
            include/graph_optimization.hpp
            )


add_executable(${PROJECT_NAME} ${Sources} )
target_link_libraries(${PROJECT_NAME} 
                      ${OpenCV_LIBS} 
                      Eigen3::Eigen 
                      ${Pangolin_LIBRARIES} glad::glad glfw
                      Ceres::ceres
                      Sophus::Sophus
                      g2o::core g2o::stuff g2o::types_sba g2o::solver_csparse g2o::csparse_extension g2o::types_icp g2o::types_slam3d g2o::types_slam2d g2o::types_data
                      ${PYTHON_LIBRARIES}
                      #pthread
                      Python3::NumPy
                      ) #