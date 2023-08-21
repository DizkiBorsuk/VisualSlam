#pragma once 
#include "common_includes.hpp"
#include "frame.hpp"

/*
Map and MapPoint classes used to creat map
*/

namespace mrVSLAM
{
    class MapPoint
    {
    // class representing 3D point in a map 
    public: 
        //### data members ###// 
        unsigned int  id = 0; // map point id 
        unsigned int frames_in = 0; // how many times/frames point was observed
        std::array<float, 3> position {}; // position of point in X,Y,Z 
        // maybe std::list instead of vector? // bjourne says noo 
        std::vector<std::weak_ptr<Feature>> point_features; // features that describe point, set of features in which point was seen 

        std::mutex point_mutex; 

        //### function members ###// 
        MapPoint(unsigned int id, std::array<float,3> point_position) noexcept; 

        std::array<float, 3> getPosition(); 
        void SetPosition(const std::array<float, 3> &point_position); 

        void addFeature(std::shared_ptr<Feature> feature); // add feature in wich point was observed, can be more that one 
        void removeFeature(std::shared_ptr<Feature> feature); 

        static std::shared_ptr<MapPoint> createMapPoint(); 

    private:

    }; 

    class Map
    {
    public: 
        //### data members ###// 
        std::mutex map_mutex; 
        std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> landmarks; // map points in map 
        std::unordered_map<unsigned int, std::shared_ptr<Frame>> keyFrames; // frames in map 
        
        std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> enabled_landmarks; //maybe active? 
        std::unordered_map<unsigned int, std::shared_ptr<Frame>> enabled_keyframes; 

        std::shared_ptr<Frame> currentFrame = nullptr; 
        unsigned int num_of_enabled_keyframes = 5; 

        //### function members ###// 
        Map() noexcept {};

        void insertKeyFrame(std::shared_ptr<Frame> frame);
        void insertPoint(std::shared_ptr<MapPoint> point);   

        void CleanMap(); 
        void removeKeyFrame(); // function for removing old/unnecessary keyframes 

    private: 

    }; 
}

