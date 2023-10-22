#include "myslam/mappoint.h"
#include "myslam/frame.h"

namespace myslam {

MapPoint::MapPoint(long id, Eigen::Vector3d position) : id_(id), pos_(position) 
{}

std::shared_ptr<MapPoint> MapPoint::CreateNewMappoint() 
{
    static long factory_id = 0;
    std::shared_ptr<MapPoint> new_mappoint(new MapPoint);
    new_mappoint->id_ = factory_id++;
    return new_mappoint;
}

void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat) {
    std::unique_lock<std::mutex> lock(mappoint_mutex);
    for (auto iter = observations_.begin(); iter != observations_.end();
         iter++) {
        if (iter->lock() == feat) {
            observations_.erase(iter);
            feat->map_point_.reset();
            observed_times_--;
            break;
        }
    }
}

}  // namespace myslam
