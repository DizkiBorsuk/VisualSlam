cmake_minimum_required(VERSION 3.15)
project(visual_slam_cpp VERSION 0.1.0)

include(CTest)
enable_testing()

set(CMAKE_TOOLCHAIN_FILE "C:/Users/Maciek/Desktop/dev_workspace/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(CMAKE_PREFIX_PATH "C:/Users/Maciek/Desktop/dev_workspace/vcpkg/installed/x64-windows/share")

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
find_package(Eigen3  REQUIRED NO_MODULE)

add_executable(${PROJECT_NAME} main.cpp visulaize_data.cpp readDataset.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Eigen3::Eigen ${Pangolin_LIBRARIES})


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)