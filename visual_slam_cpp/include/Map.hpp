#pragma once 
#include <vector>
#include <memory>

namespace mrVSLAM
{
    class MapPoint
    {   
        // 3D point in a world 
        std::shared_ptr<MapPoint> mapPointPtr; 

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