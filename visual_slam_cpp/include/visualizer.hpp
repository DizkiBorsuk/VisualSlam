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

        void setVisualizer(std::shared_ptr<Map> ptr_to_map, Eigen::Matrix3d K)
        {
            map = ptr_to_map; 

            cx = K.coeff(0,2); 
            cy = K.coeff(1,2); 
            fx = K.coeff(0,0); 
            fy = K.coeff(1,1); 
        }

       void addNewFrame(std::shared_ptr<Frame> in_frame); 
       void getMapUpdate();

    private:  

        std::thread visualizer_thread;
        std::mutex visualizer_mutex; 

        std::shared_ptr<Frame> current_frame = nullptr;
        std::shared_ptr<Map> map = nullptr; 

        void runVisualizer(); 

        void drawFrame(std::shared_ptr<Frame> input_frame, const std::array<float,3> color); 
        void drawPoints(const std::array<float,3> color); 
        void drawFrameTrajectory(pangolin::OpenGlRenderState& s_cam); 
        

        std::unordered_map<unsigned int, std::shared_ptr<Frame>> displayed_keyframes; 
        std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> displayed_mappoints; 

        // OpenGl stuff 

        static constexpr int window_w = 640; 
        static constexpr int window_h= 480; 
        double fx =0; 
        double fy = 0; 
        double cy = 0; 
        double cx = 0; 

        static constexpr std::array<float,3> blue = {0,0,1}; 
        static constexpr std::array<float,3> green = {0,1,0};
        static constexpr std::array<float,3> red = {1,0,0};  
        static constexpr std::array<float,3> magenta = {1,0,1};
        static constexpr std::array<float,3> yellow = {1,1,0};
        static constexpr float line_width = 2.0; 

    }; 
}

