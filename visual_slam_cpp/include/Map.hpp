#pragma once
#include "system.hpp" 
#include "FrameExtraction.hpp"

namespace mrVSLAM
{
    class Point
    {   
        // class that represents a 3D point in a world 
        std::shared_ptr<Point> mapPointPtr; 
        std::array<float, 3> pointXYZ {}; // position of point in x,y,z 
        cv::Mat pointDescriptor; 

        unsigned int point_id = 0; 
        unsigned int observed_in = 0; // number of frames that point was observed in 

        Point(int id, std::array<float,3> position);

    }; 


    class Map
    {
        // Map is colection of points in 3D/spacial points 
    public: 
        std::unordered_map<int, std::shared_ptr<Point>> mapPoints; // landmarks in map 
        std::unordered_map<int, std::shared_ptr<Frame>> mapFrames; // keyframes in map 

        Map(); 
        void insertMapPoint(std::shared_ptr<Point> ptr_to_mapPoint); // function that adds points to map 
        void insertFrame(); // adds new frame to map
        void CleanMap();

        std::unordered_map<int, std::shared_ptr<Point>> getMapPoints(); 

        void removeOldKeyframe(); 

    }; 
}