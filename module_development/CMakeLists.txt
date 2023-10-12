cmake_minimum_required(VERSION 3.20)
project(module_development VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_BUILD_TYPE "Release")


set(CMAKE_CXX_FLAGS "-std=c++20 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++20 -O3 -fopenmp -pthread") 


IF(WIN32)
    set(CMAKE_TOOLCHAIN_FILE "C:/Users/Maciek/Desktop/dev_workspace/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_PREFIX_PATH "C:/Users/Maciek/Desktop/dev_workspace/vcpkg/installed/x64-windows/share")

ENDIF()


find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3  REQUIRED NO_MODULE)
# find_package(Ceres REQUIRED)
find_package(Sophus CONFIG REQUIRED)


include_directories( ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS} )
#include_directories(SYSTEM ${matplotlib_SOURCE_DIR})

set(Sources main.cpp 
            
            )

# SET(G2O_libs )

add_executable(${PROJECT_NAME} ${Sources} )
target_link_libraries(${PROJECT_NAME} 
                      ${OpenCV_LIBS} 
                      Eigen3::Eigen 
                      ${Pangolin_LIBRARIES} 
                    #   Ceres::ceres
                      Sophus::Sophus
                      ) #