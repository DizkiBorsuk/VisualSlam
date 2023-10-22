#pragma once 

#include <pangolin/pangolin.h>
#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {

    class Visualizer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Visualizer(bool show_whole_map = false);

        void SetMap(std::shared_ptr<Map> map_ptr) 
        { 
            map = map_ptr; 
        }

        void Close();
        void AddCurrentFrame(std::shared_ptr<Frame> current_frame);
        void UpdateMap();

    private:
        void ThreadLoop();
        void DrawFrame(std::shared_ptr<Frame> frame, const int* frame_color);
        void DrawMapPoints(const int* point_color);
        void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

        /// plot the features in current frame into an image
        cv::Mat PlotFrameImage();

        std::shared_ptr<Frame> current_frame_ = nullptr;
        std::shared_ptr<Map> map = nullptr;

        bool visualizer_running = true;
        bool whole_map = false; 
        bool map_updated = false;

        std::unordered_map<unsigned long, std::shared_ptr<Frame>> keyframes;
        std::unordered_map<unsigned long, std::shared_ptr<MapPoint>> landmarks;
      
        std::thread visualizer_thread;
        std::mutex visualizer_mutex;

        static constexpr float sz = 1.0;
        static constexpr int line_width = 2.0;
        static constexpr float fx = 400;
        static constexpr float fy = 400;
        static constexpr float cx = 512;
        static constexpr float cy = 384;
        static constexpr float width = 1024;
        static constexpr float height = 768;


        static constexpr int blue[3] = {0,0,1}; 
        static constexpr int green[3] = {0,1,0};
        static constexpr int red[3] = {1,0,0};  
        static constexpr int magenta[3] = {1,0,1};
        static constexpr int yellow[3] = {1,1,0};
    };
}  
