/**
 * @file map.cpp
 * @author mrostocki
 * @brief 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "mrVSLAM/map.hpp" 
#include "mrVSLAM/frame.hpp"

namespace mrVSLAM
{
    void Map::insertNewKeyframe(std::shared_ptr<Frame> frame)
    {
        this->current_keyframe = frame; 

        {
            std::unique_lock<std::mutex> lock(map_mutex); //! look into locking and unlocking locks 
        
            if (!activeKeyframesDictionary.contains(frame->kf_id))  //keyframes_.find(frame->keyframe_id) == keyframes_.end()
            {
                allKeyframesDictionary.insert(make_pair(frame->kf_id, frame));
                activeKeyframesDictionary.insert(make_pair(frame->kf_id, frame));
            } else {
                allKeyframesDictionary[frame->kf_id] = frame;
                activeKeyframesDictionary[frame->kf_id] = frame;
            }
        }
        
        if (activeKeyframesDictionary.size() > this->num_of_active_keyframes)
        {
            removeOldKeyframe();
        }

    }

    void Map::insertNewMappoint(std::shared_ptr<MapPoint> map_point)
    {

    }

    std::shared_ptr<Frame> Map::getKyeframeById(unsigned int keyframe_id)
    {
        std::unique_lock<std::mutex> lock(map_mutex);
        if(allKeyframesDictionary.contains(keyframe_id))
            return allKeyframesDictionary.at(keyframe_id); 
        else 
            std::cerr << "Map::getKyeframeById() - frame doesn't exist \n"; 

        return nullptr; 
    }

    void Map::cleanMap()
    {

    }

    void Map::removeOldKeyframe()
    {

    }


} //! end of namespace