#include "../include/map.hpp"

namespace mrVSLAM
{
    MapPoint::MapPoint(unsigned int id, std::array<float,3> point_position) noexcept
        : id(id), position(point_position)
    {

    }

    std::array<float,3> MapPoint::getPosition()
    {
        std::lock_guard<std::mutex> lock(point_mutex); 
        return position; 
    }

    void MapPoint::setPosition(const std::array<float, 3> &point_position)
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

    }

    // ########## // 

    void Map::insertKeyFrame(std::shared_ptr<Frame> keyframe)
    {
        currentFrame = keyframe; 
        if(keyFrames.contains(keyframe->keyframe_id)) // check if hashmap already contains said frame with that id
        {
            //if hashmap already contains frame with that id replace pointer to it? 
            //? not sure about that 
            

        } 
        else 
        {
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
        if(landmarks.contains(mappoint->id))
        {
            //? 
            landmarks[mappoint->id] = mappoint; 
            enabled_landmarks[mappoint->id] = mappoint
        }
        else 
        {
            landmarks.insert({mappoint->id, mappoint});
            enabled_landmarks.insert({mappoint->id, mappoint});
        }
    }

    void Map::cleanMap()
    {

    }

    void Map::removeOldestKeyFrame()
    {

    }


}