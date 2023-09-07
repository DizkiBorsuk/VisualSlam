#pragma once 
#include "common_includes.hpp"
#include "frame.hpp"

/*
Map and MapPoint classes used to creat map. 
MapPoint should store: 3D position, representative descriptor and ... 
*/

namespace mrVSLAM
{
    class MapPoint
    {
    // class representing 3D point in a map 
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        //### data members ###// 
        unsigned int  id = 0; // map point id 
        unsigned int frames_in = 0; // how many times/frames point was observed during matching 
        std::array<float, 3> position {}; // position of point in X,Y,Z 
        //? maybe std::list instead of vector? // bjourne says noo 
        std::vector<std::weak_ptr<Feature>> point_features; // features that describe point, set of features in which point was seen 

        std::mutex point_mutex; 

        //### function members ###// 
        MapPoint(unsigned int id, std::array<float,3> point_position) noexcept; 

        std::array<float, 3> getPosition(); 
        void setPosition(const std::array<float, 3> &point_position); 

        void addFeature(std::shared_ptr<Feature> feature); // add feature in wich point was observed, can be more that one 
        void removeFeature(std::shared_ptr<Feature> feature); 

        static std::shared_ptr<MapPoint> createMapPoint(); 

    private:

    }; 

    class Map
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        //### data members ###// 
        std::mutex map_mutex; 
        std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> landmarks; // map points in map 
        std::unordered_map<unsigned int, std::shared_ptr<Frame>> keyFrames; // frames in map 
        
        std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> enabled_landmarks; //?maybe active? 
        std::unordered_map<unsigned int, std::shared_ptr<Frame>> enabled_keyframes; 

        std::shared_ptr<Frame> currentFrame = nullptr; 
        unsigned int num_of_enabled_keyframes = 5; 

        //### function members ###// 
        Map() noexcept = default;

        void insertKeyFrame(std::shared_ptr<Frame> keyframe);
        void insertPoint(std::shared_ptr<MapPoint> mappoint);   

        void cleanMap(); 
        void removeOldestKeyFrame(); // function for removing old/unnecessary keyframes 

        // 
        std::unordered_map<unsigned int, std::shared_ptr<Frame>> getAllKeyframes(); // get frame and landmarks thread safe for bundle adjustment 
        std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> getAllMappoints(); 
        std::unordered_map<unsigned int, std::shared_ptr<Frame>> getEnabledKeyframes(); 
        std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> getEnabledMappoints(); 

        unsigned int getNumberOfPointsInMap(); 
        unsigned int getNumberOfFramesInMap(); 

    private: 

    }; 
}

