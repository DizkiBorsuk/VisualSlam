#pragma once
#include "myslam/common_include.hpp"

namespace myslam {

    class Frame;
    class Feature;

    class MapPoint 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        unsigned long id_ = 0;  // ID
        bool is_outlier_ = false;
        Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();  // Position in world
        std::mutex mappoint_mutex;
        int observed_times_ = 0;  // being observed by feature matching algo.
        std::list<std::weak_ptr<Feature>> observations_;

        MapPoint() {}

        MapPoint(long id, Eigen::Vector3d position);

        Eigen::Vector3d Pos() {
            std::unique_lock<std::mutex> lock(mappoint_mutex);
            return pos_;
        }

        void SetPos(const Eigen::Vector3d &pos) {
            std::unique_lock<std::mutex> lock(mappoint_mutex);
            pos_ = pos;
        };

        void AddObservation(std::shared_ptr<Feature> feature) 
        {
            std::unique_lock<std::mutex> lock(mappoint_mutex);
            observations_.push_back(feature);
            observed_times_++;
        }

        void RemoveObservation(std::shared_ptr<Feature> feat);

        std::list<std::weak_ptr<Feature>> GetObs() 
        {
            std::unique_lock<std::mutex> lock(mappoint_mutex);
            return observations_;
        }

        // factory function
        static std::shared_ptr<MapPoint> CreateNewMappoint();
    };
}  
