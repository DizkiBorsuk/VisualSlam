#include "../include/map.hpp"

namespace mrVSLAM
{
    MapPoint::MapPoint(unsigned int in_id, Eigen::Vector3d point_position) noexcept
        : id(in_id), position(point_position)
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
        currentKeyFrame = keyframe; 

        if(keyFrames.contains(keyframe->keyframe_id)) // check if hashmap already contains said frame with that id
        {
            std::cout << "map already contains this keyframe id:" << currentKeyFrame->keyframe_id; 
            // keyFrames[keyframe->keyframe_id] = keyframe; 
            // enabled_keyframes[keyframe->keyframe_id] = keyframe; 
            
        } else {
            //if not, put it in keyframes map 
            std::cout << "map does not contains this keyframe \n"; 
            keyFrames.insert({keyframe->keyframe_id, keyframe}); 
            enabled_keyframes.insert({keyframe->keyframe_id, keyframe});
        }

        std::cout << "number of keyframes in map " << keyFrames.size() << "\n";

        if(enabled_keyframes.size() > num_of_enabled_keyframes)
        {
            removeOldestKeyFrame(); 
        }
    }

    void Map::insertPoint(std::shared_ptr<MapPoint> mappoint)
    {
        if(mappoints.contains(mappoint->id))
        {
            // mappoints[mappoint->id] = mappoint; 
            // enabled_mappoints[mappoint->id] = mappoint;
            std::cout << "map with this id already exist \n"; 
        } else {
            mappoints.insert({mappoint->id, mappoint});
            enabled_mappoints.insert({mappoint->id, mappoint});
        }

        std::cout << "map contains " << mappoints.size() << " mappoints \n"; 
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
        std::shared_ptr<Frame> oldest_frame = nullptr; 

        auto Tmatrix_wc = currentKeyFrame->getSophusFramePose().inverse(); 
        double max_kf_distance = 0; 
        unsigned int max_dis_kf_id = 0; 
        double min_kf_distance = 10000; //some big valuse  
        unsigned int min_dis_kf_id = 0; 

        for(auto &frame : enabled_keyframes)
        {
            if(frame.second == currentKeyFrame)
            {
                continue; // don't want to remove current frame 
            }
            else 
            {
                double distance = (frame.second->getSophusFramePose()*Tmatrix_wc).log().norm(); 

                if(distance > max_kf_distance)
                {
                    max_kf_distance = distance; //find max distance of keyframe form current frame
                    max_dis_kf_id = frame.first; 
                } 

                if(distance < min_kf_distance)
                {
                    min_kf_distance = distance; 
                    min_dis_kf_id = frame.first; 
                }
            }
        }

        double dist_treshold = 0.2; // to be checked 
        if(min_kf_distance < dist_treshold)
        {
            oldest_frame = keyFrames.at(min_dis_kf_id); 
        }
        else 
        {
            oldest_frame = keyFrames.at(max_dis_kf_id); 
        }

        std::cout << "remove frame : " << oldest_frame->keyframe_id << "\n"; 

        enabled_keyframes.erase(oldest_frame->keyframe_id); 

        for(auto &feature : oldest_frame->featuresFromLeftImg) 
        {
            auto point = feature->map_point.lock(); 
            if(point != nullptr)
            {
                point->removeFeature(feature); 
            }
        }

        cleanMap(); 
    }

    unsigned int Map::getNumberOfPointsInMap()
    {
        std::lock_guard<std::mutex> lock(map_mutex); 
        return mappoints.size(); 

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
        return mappoints; 
    }
    std::unordered_map<unsigned int, std::shared_ptr<Frame>> Map::getEnabledKeyframes()
    {
        std::lock_guard<std::mutex> lock(map_mutex); 
        return enabled_keyframes; 
    }
    std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> Map::getEnabledMappoints()
    {
        std::lock_guard<std::mutex> lock(map_mutex); 
        return enabled_mappoints; 
    }
}