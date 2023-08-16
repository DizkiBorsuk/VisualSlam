#pragma once
#include "system.hpp" 


namespace mrVSLAM
{
    class MapPoint
    {   
        // 3D point in a world 
        std::shared_ptr<MapPoint> mapPointPtr; 
        std::array<float, 3> pointXYZ; 
        cv::Mat pointDescriptor; 

    }; 


    class Map
    {
        // Map is colection of points in 3D/spacial points 
    public: 
        std::vector<mrVSLAM::MapPoint> points; 

        Map(); 
        void insertMapPoint(); 
        void CleanMap(); 

    }; 
}