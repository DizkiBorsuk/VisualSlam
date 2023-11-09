add_library(myslam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        stereo_tracking.cpp
        local_mapping.cpp
        visualizer.cpp
        slam.cpp
        read_dataset.cpp
        tools.cpp
        )

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})
