#pragma once

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam {

    class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> LandmarksType;
        typedef std::unordered_map<unsigned int, std::shared_ptr<Frame>> KeyframesType;

        Map() {}

        void InsertKeyFrame(std::shared_ptr<Frame> frame);
        void InsertMapPoint(std::shared_ptr<MapPoint> map_point);

        LandmarksType GetAllMapPoints() {
            std::unique_lock<std::mutex> lck(map_mutex);
            return landmarks_;
        }
        KeyframesType GetAllKeyFrames() {
            std::unique_lock<std::mutex> lck(map_mutex);
            return keyframes_;
        }
        LandmarksType GetActiveMapPoints() {
            std::unique_lock<std::mutex> lck(map_mutex);
            return active_landmarks_;
        }

        KeyframesType GetActiveKeyFrames() {
            std::unique_lock<std::mutex> lck(map_mutex);
            return active_keyframes_;
        }

        void CleanMap();

    private:
        void RemoveOldKeyframe();

        std::mutex map_mutex;
        LandmarksType landmarks_;         // all landmarks
        LandmarksType active_landmarks_;  // active landmarks
        KeyframesType keyframes_;         // all keyframes
        KeyframesType active_keyframes_;  // active keyframes

        std::shared_ptr<Frame> current_frame_ = nullptr;
        int num_active_keyframes = 10;  
    };
}  


