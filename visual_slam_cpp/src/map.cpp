#include "../include/map.hpp"

namespace mrVSLAM
{
    MapPoint::MapPoint(unsigned int id, Eigen::Vector3d point_position) noexcept
        : id(id), position(point_position)
    {
        mappoint_counter++; 
    }

    Eigen::Vector3d MapPoint::getPosition()
    {
        std::lock_guard<std::mutex> lock(point_mutex); 
        return position; 
    }

    void MapPoint::setPosition(const Eigen::Vector3d &point_position)
    {
        std::lock_guard<std::mutex> lock(point_mutex); 
        position = point_position; 
    }

    void MapPoint::addFeature(std::shared_ptr<Feature> feature)
    {
        // andd Feature object to point 
        std::lock_guard<std::mutex> lock(point_mutex); 
        point_features.emplace_back(feature); 
        frames_in++; 
    }

    void MapPoint::removeFeature(std::shared_ptr<Feature> feature)
    {
        std::lock_guard<std::mutex> lock(point_mutex); 
        for(auto it = point_features.begin(); it != point_features.end(); it++)
        {
            if(it->lock() == feature)
            {
                point_features.erase(it); 
                feature->map_point.reset(); 
                frames_in--; 
                break; 
            }
        }
    }

    std::list<std::weak_ptr<Feature>> MapPoint::getFeatures()
    {
        std::lock_guard<std::mutex> lock(point_mutex);
        return point_features; 
    }

    // ################################################################################## // 

    void Map::insertKeyFrame(std::shared_ptr<Frame> keyframe)
    {
        currentFrame = keyframe; 

        if(keyFrames.contains(keyframe->keyframe_id)) // check if hashmap already contains said frame with that id
        {
            //if hashmap already contains frame with that id replace pointer to it? 
            //? not sure about that 
            

        } else {
            //if not, put it in keyframes map 
            keyFrames.insert({keyframe->keyframe_id, keyframe}); 
            enabled_keyframes.insert({keyframe->keyframe_id, keyframe});
        }

        if(enabled_keyframes.size() > num_of_enabled_keyframes)
        {
            removeOldestKeyFrame(); 
        }
    }

    void Map::insertPoint(std::shared_ptr<MapPoint> mappoint)
    {
        if(mappoints.contains(mappoint->id))
        {
            //? 
            mappoints[mappoint->id] = mappoint; 
            enabled_mappoints[mappoint->id] = mappoint;
        } else {
            mappoints.insert({mappoint->id, mappoint});
            enabled_mappoints.insert({mappoint->id, mappoint});
        }
    }

    void Map::cleanMap()
    {
        /*
        remove mappoints from active mappoints
        */
        unsigned int mappoints_removed = 0;

        for (auto it = enabled_mappoints.begin(); it != enabled_mappoints.end();) 
        {
            if (it->second->frames_in == 0) 
            {
                it = enabled_mappoints.erase(it);
                mappoints_removed++; 
            } else {
                it++; 
            }
        }
        std::cout << "Removed " << mappoints_removed << " active landmarks";
    }

    void Map::removeOldestKeyFrame()
    {

    }

    unsigned int Map::getNumberOfPointsInMap()
    {
        std::lock_guard<std::mutex> lock(map_mutex); 
        return landmarks.size(); 

    } 
    unsigned int Map::getNumberOfFramesInMap()
    {
        std::lock_guard<std::mutex> lock(map_mutex); 
        return keyFrames.size();  
    }

    std::unordered_map<unsigned int, std::shared_ptr<Frame>> Map::getAllKeyframes() 
    {
        std::lock_guard<std::mutex> lock(map_mutex); 
        return keyFrames; 
    }
    std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> Map::getAllMappoints()
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        return landmarks; 
    }
    std::unordered_map<unsigned int, std::shared_ptr<Frame>> Map::getEnabledKeyframes()
    {
        std::lock_guard<std::mutex> lock(map_mutex); 
        return enabled_keyframes; 
    }
    std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> Map::getEnabledMappoints()
    {
        std::lock_guard<std::mutex> lock(map_mutex); 
        return enabled_landmarks; 
    }
}