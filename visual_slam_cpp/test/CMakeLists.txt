
set(This slamTests)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIR})

#######################
add_executable(keyframe_test keyframe_test.cpp)
target_link_libraries(keyframe_test PUBLIC 
                      gtest_main
                      gtest
                      mrVSLAM )

add_test(NAME keyframe_test
         COMMAND keyframe_test
        )     
#######################

#######################
add_executable(mappoint_test mappoint_test.cpp)
target_link_libraries(mappoint_test PUBLIC 
                      gtest_main
                      gtest
                      mrVSLAM )

add_test(NAME mappoint_test
        COMMAND mappoint_test
        )     
#######################

#######################
add_executable(slam_test slam_tests.cpp)
target_link_libraries(slam_test PUBLIC 
                      gtest_main
                      gtest
                      mrVSLAM )

add_test(NAME slam_test
        COMMAND slam_test
        )     
#######################