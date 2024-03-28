/**
 * @file visualizer.hpp
 * @author mrostocki
 * @brief map and camera visualization thread/module 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once 
#include "mrVSLAM/common_includes.hpp" 
#include <pangolin/pangolin.h>

namespace mrVSLAM
{   
    class Frame; 
    class KeyFrame; 
    class MapPoint; 
    class Map; 

    /**
     * @class Visualizer 
     * @brief Visualizer class that handles map and img visualization 
     */
    class Visualizer
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 

        /**
         * @brief Construct a new Visualizer object
         * 
         * @param[in] show_whole_map if false only active map will be showed (false by default)
         * @param[in] show_img if true current left camera img with detected features will be showed (true by default)
         * @param[in] show_matching_points if true camera img will also contain visualization of tracked points 
         */
        Visualizer(bool show_whole_map = false, bool show_img = true, bool show_matching_points = false); 
        
        void setupVisualizer(std::shared_ptr<Map> map_ptr)
        {
            this->map = map_ptr; 
        }

        /**
         * @brief add new frame to visualizer loop 
         * @param[in] frame - ptr to frame object 
         */
        void addNewFrame(std::shared_ptr<Frame> frame)
        {
            std::unique_lock<std::mutex> lock(visualizer_mutex);
            current_frame = frame; 
        }

        /**
         * @brief end visualizer thread
         */
        void close(); 

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool updateMap(); 

    private: 

        void runVisualizerThread(); 
        void drawFrame(std::shared_ptr<Frame> frame, const int* frame_color);  
        void drawMap(const int* point_color); 
        void followNewFrame(pangolin::OpenGlRenderState& vis_camera); 
        cv::Mat drawFrameImg(bool draw_matching_points = false); 

    private: 

        std::thread visualizer_thread;
        std::mutex visualizer_mutex;

        std::shared_ptr<Frame> current_frame = nullptr;
        std::shared_ptr<Frame> prev_frame = nullptr; 
        std::shared_ptr<Map> map = nullptr;

        std::unordered_map<unsigned int, std::shared_ptr<KeyFrame>> keyframes; 
        std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> mappoints; 

        static constexpr float sz = 1.0;
        static constexpr float fx = 400;
        static constexpr float fy = 400;
        static constexpr float cx = 512;
        static constexpr float cy = 384;
        static constexpr float width = 1024;
        static constexpr float height = 768;
        static constexpr int line_width = 2.0;

        static constexpr int blue[3] = {0,0,1}; 
        static constexpr int green[3] = {0,1,0};
        static constexpr int red[3] = {1,0,0};  
        static constexpr int magenta[3] = {1,0,1};
        static constexpr int yellow[3] = {1,1,0};

        bool visualizer_running = true;
        bool whole_map = false; 
        bool show_img = true; 
        bool show_matching_points = false; 
    }; 

} //! end of namespace 