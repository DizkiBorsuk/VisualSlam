#pragma once 
#include "common_includes.hpp"
#include "map.hpp"
#include "frame.hpp"


namespace mrVSLAM
{
    class Visualizer
    {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 

        Visualizer(); 

        void setMapPtr(std::shared_ptr<Map> input_map); 

    private:  

        std::thread visualizer_thread;
        std::shared_ptr<Frame> current_frame = nullptr;
        std::shared_ptr<Map> ptr_to_map = nullptr; 


        void drawFrame(std::shared_ptr<Frame> input_frame); 
        void drawPoints(); 
        void drawFrameTrajectory(); 





    }; 

}

