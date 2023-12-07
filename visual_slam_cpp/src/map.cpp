#include "myslam/map.hpp"

namespace myslam 
{
    void Map::InsertKeyFrame(std::shared_ptr<Frame> frame) 
    {
        current_frame_ = frame;
        if (!keyframes.contains(frame->keyframe_id))  //keyframes_.find(frame->keyframe_id) == keyframes_.end()
        {
            keyframes.insert(make_pair(frame->keyframe_id, frame));
            active_keyframes.insert(make_pair(frame->keyframe_id, frame));
        } else {
            keyframes[frame->keyframe_id] = frame;
            active_keyframes[frame->keyframe_id] = frame;
        }

        if (active_keyframes.size() > num_active_keyframes)
        {
            RemoveOldKeyframe();
        }
    }

    void Map::InsertMapPoint(std::shared_ptr<MapPoint> map_point) 
    {
        if (!landmarks.contains(map_point->id_))  //landmarks_.find(map_point->id_) == landmarks_.end()
        {
            landmarks.insert(make_pair(map_point->id_, map_point));
            active_landmarks.insert(make_pair(map_point->id_, map_point));
        } 
        else 
        {
            landmarks[map_point->id_] = map_point;
            active_landmarks[map_point->id_] = map_point;
        }
    }

    void Map::RemoveOldKeyframe() 
    {
        if (current_frame_ == nullptr) 
        {
            return;
        }

        double max_dis = 0, min_dis = 9999;
        double max_kf_id = 0, min_kf_id = 0;
        auto Twc = current_frame_->Pose().inverse();

        for (auto& kf : active_keyframes) 
        {
            if (kf.second == current_frame_)
            {
                continue;
            }
            auto dis = (kf.second->Pose() * Twc).log().norm();
            if (dis > max_dis) 
            {
                max_dis = dis;
                max_kf_id = kf.first;
            }
            if (dis < min_dis) 
            {
                min_dis = dis;
                min_kf_id = kf.first;
            }
        }

        const double min_dis_th = 0.2;  
        std::shared_ptr<Frame> frame_to_remove = nullptr;
        if (min_dis < min_dis_th) 
        {
            frame_to_remove = keyframes.at(min_kf_id);
        } else {
            frame_to_remove = keyframes.at(max_kf_id);
        }

        std::cout  << "remove keyframe " << frame_to_remove->keyframe_id;
        // remove keyframe and landmark observation
        active_keyframes.erase(frame_to_remove->keyframe_id);

        for (auto feature : frame_to_remove->features_left_) 
        {
            auto temp_mappoint = feature->map_point_.lock();
            if (temp_mappoint) 
            {
                temp_mappoint->RemoveObservation(feature);
            }
        }

        for (auto feature : frame_to_remove->features_right_) 
        {
            if (feature == nullptr)
            { 
                continue;
            }
            auto temp_mappoint = feature->map_point_.lock();
            if (temp_mappoint) 
            {
                temp_mappoint->RemoveObservation(feature);
            }
        }
        CleanMap();
    }

    void Map::CleanMap() {
        int cnt_landmark_removed = 0;
        for (auto iter = active_landmarks.begin(); iter != active_landmarks.end();) 
        {
            if (iter->second->observed_times_ == 0)
            {
                iter = active_landmarks.erase(iter);
                cnt_landmark_removed++;
            } else {
                ++iter;
            }
        }
        std::cout  << "Removed " << cnt_landmark_removed << " active landmarks \n";
    }

}  // namespace myslam
