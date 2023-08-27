#pragma once 
#include "common_includes.hpp"
#include "map.hpp"
#include "frame.hpp"

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/gldraw.h> 

namespace mrVSLAM
{
    class Visualizer
    {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 

        Visualizer(); //* constructor creates visualization thread 
        void closeVisualizer(); //* closes/joints thread 

        void setMapPtr(std::shared_ptr<Map> ptr_to_map)
        {
            map = ptr_to_map; 
        }

       void addNewFrame(std::shared_ptr<Frame> in_frame); 
       

    private:  

        std::thread visualizer_thread;
        std::mutex visualizer_mutex; 

        std::shared_ptr<Frame> current_frame = nullptr;
        std::shared_ptr<Map> map = nullptr; 

        void runVisualizer(); 

        void drawFrame(std::shared_ptr<Frame> input_frame); 
        void drawPoints(); 
        void drawFrameTrajectory(pangolin::OpenGlRenderState& s_cam); 
        void getMapUpdate(); 

        std::unordered_map<unsigned int, std::shared_ptr<Frame>> displayed_keyframes; 
        std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> displayed_mappoints; 
    }; 
}

