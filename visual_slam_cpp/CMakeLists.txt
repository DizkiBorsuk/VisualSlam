cmake_minimum_required(VERSION 3.15)
project(myslam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-std=c++20 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++20 -O3 -fopenmp -pthread")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )

############### dependencies ######################
find_package(Eigen3  REQUIRED NO_MODULE)
find_package(OpenCV  REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(Matplot++ REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS}
                    ${Sophus_INCLUDE_DIRS}
                    ${G2O_INCLUDE_DIRS}
                    )

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${DBoW3_LIBS}
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        pthread
        Eigen3::Eigen 
        Matplot++::matplot
        )

include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
# add_subdirectory(DBow3)

add_executable(run_SLAM main.cpp)
target_link_libraries(run_SLAM myslam )
