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
#include "mrVSLAM/mappoint.hpp"

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

        for(auto &feature : frame->features_on_left_img){
            auto mp = feature->map_point.lock(); 
            if(mp) {
                mp->addActiveObservation(feature); 
            }
        }
        
        if (activeKeyframesDictionary.size() > this->num_of_active_keyframes)
        {
            removeOldKeyframeAndMappoints();
        }

    }

    void Map::insertNewMappoint(std::shared_ptr<MapPoint> map_point)
    {
        std::unique_lock<std::mutex> lock(map_mutex);

        if (!allMappointsDictionary.contains(map_point->id))  //landmarks_.find(map_point->id_) == landmarks_.end()
        {
            allMappointsDictionary.insert(make_pair(map_point->id, map_point));
            activeMappointsDictionary.insert(make_pair(map_point->id, map_point));
        } 
        else 
        {
            allMappointsDictionary[map_point->id] = map_point;
            activeMappointsDictionary[map_point->id] = map_point;
        }
    }

    std::shared_ptr<Frame> Map::getKyeframeById(unsigned int keyframe_id)
    {
        std::unique_lock<std::mutex> lock(map_mutex);
        if(allKeyframesDictionary.contains(keyframe_id))
            return allKeyframesDictionary.at(keyframe_id); 
        else 
            fmt::print(fg(fmt::color::dark_red), "Map::getKyeframeById() - frame doesn't exist \n"); 

        return nullptr; 
    }

    void Map::removeOldKeyframeAndMappoints()
    {
        std::unique_lock<std::mutex> lock(map_mutex);

        if (current_keyframe == nullptr) {
            return;
        }

        double max_dis = 0, min_dis = 9999;
        double max_kf_id = 0, min_kf_id = 0;
        auto Twc = current_keyframe->getPose().inverse();

        for (auto& kf : activeKeyframesDictionary) 
        {
            if (kf.second == current_keyframe) {
                continue;
            }

            // get distance of n keyframe from current keyframe
            auto dis = (kf.second->getPose() * Twc).log().norm();  
            if (dis > max_dis) {
                max_dis = dis;
                max_kf_id = kf.first;
            }
            if (dis < min_dis) {
                min_dis = dis;
                min_kf_id = kf.first;
            }
        }

        const double min_dis_th = 0.2;  
        std::shared_ptr<Frame> frame_to_remove = nullptr;
        if (min_dis < min_dis_th) 
        {
            frame_to_remove = allKeyframesDictionary.at(min_kf_id);
        } else {
            frame_to_remove = allKeyframesDictionary.at(max_kf_id);
        }

        fmt::print(fg(fmt::color::yellow), "removed keyframe {} \n", frame_to_remove->kf_id); 
        // remove keyframe and landmark observation
        activeKeyframesDictionary.erase(frame_to_remove->kf_id);

        for (auto feature : frame_to_remove->features_on_left_img) 
        {
            auto temp_mappoint = feature->map_point.lock();
            if (temp_mappoint) 
            {
                temp_mappoint->removeActiveObservation(feature);
            }
        }

        for (auto feature : frame_to_remove->features_on_right_img) 
        {
            if (feature == nullptr)
            { 
                continue;
            }
            auto temp_mappoint = feature->map_point.lock();
            if (temp_mappoint) 
            {
                temp_mappoint->removeActiveObservation(feature);
            }
        }

        int landmarks_removed = 0;
        for (auto iter = activeMappointsDictionary.begin(); iter != activeMappointsDictionary.end();) 
        {
            if (iter->second->active_observed_times == 0)
            {
                iter = activeMappointsDictionary.erase(iter);
                landmarks_removed++;
            } else {
                ++iter;
            }
        }
        fmt::print(fg(fmt::color::yellow), "Removed {} old mappoints from local map \n", landmarks_removed);
    }
    

} //! end of namespace