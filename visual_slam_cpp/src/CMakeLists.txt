add_library(mrVSLAM SHARED
                frame.cpp
                map.cpp
                camera.cpp
                stereo_tracking.cpp
                mono_tracking.cpp
                local_mapping.cpp
                loop_closing.cpp
                visualizer.cpp
                slam.cpp
                read_dataset.cpp 
        )

target_link_libraries(mrVSLAM ${THIRD_PARTY_LIBS})
