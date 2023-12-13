#pragma once

#include "myslam/common_include.hpp"
#include "myslam/frame.hpp"
#include "myslam/mappoint.hpp"

namespace myslam {

    class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> LandmarksType;
        typedef std::unordered_map<unsigned int, std::shared_ptr<Frame>> KeyframesType;

        Map() {}

        void InsertKeyFrame(std::shared_ptr<Frame> frame);
        void InsertMapPoint(std::shared_ptr<MapPoint> map_point);

        LandmarksType GetAllMapPoints() {
            std::unique_lock<std::mutex> lock(map_mutex);
            return landmarks;
        }
        KeyframesType GetAllKeyFrames() {
            std::unique_lock<std::mutex> lock(map_mutex);
            return keyframes;
        }
        LandmarksType GetActiveMapPoints() {
            std::unique_lock<std::mutex> lock(map_mutex);
            return active_landmarks;
        }

        KeyframesType GetActiveKeyFrames() {
            std::unique_lock<std::mutex> lock(map_mutex);
            return active_keyframes;
        }

        std::shared_ptr<Frame> getKeyFrameById(unsigned int kf_id)
        {
            std::unique_lock<std::mutex> lock(map_mutex);
            if(keyframes.contains(kf_id))
                return keyframes.at(kf_id); 
            else 
                std::cerr << "frame doesn't exist \n"; 

            return nullptr; 
        }


        void CleanMap();
        int getNumberOfKeyframes()
        {
            return keyframes.size(); 
        }

        std::mutex public_map_mutex; 

    private:
        void RemoveOldKeyframe();

        std::mutex map_mutex;
        LandmarksType landmarks;         
        LandmarksType active_landmarks; 
        KeyframesType keyframes;         
        KeyframesType active_keyframes;  

        std::shared_ptr<Frame> current_frame_ = nullptr;
        unsigned int num_active_keyframes = 10;  
    };
}  


