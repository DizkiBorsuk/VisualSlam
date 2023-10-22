add_library(myslam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        stereo_tracking.cpp
        local_mapping.cpp
        viewer.cpp
        slam.cpp
        dataset.cpp
        tools.cpp)

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})
