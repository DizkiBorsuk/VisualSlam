/**
 * @file mappoint.hpp
 * @author mrostocki
 * @brief mappoint class that represents observed 3D point in a map 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once 
#include "mrVSLAM/common_includes.hpp" 
#include "mrVSLAM/frame.hpp"

namespace mrVSLAM
{   
    /**
     * @class MapPoint 
     * @brief MapPoint class represents a 3D point in a map/enviornment 
     */
    class MapPoint
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  

        /**
         * @brief Default MapPoint constructor 
         * 
         */
        MapPoint() = default; 
        /**
         * @brief Construct a new MapPoint object with automatic id number 
         * @param position 3D position of point in map (X,Y,Z)
         */
        MapPoint(Eigen::Vector3d position)
        {
            static unsigned int factory_id = 0;
            this->id = ++factory_id; 
            this->poinst_pos = position; 
        }  

        void setPosition(const Eigen::Vector3d &position)
        {
            std::unique_lock<std::mutex> lock(mappoint_mutex);
            this->poinst_pos = position; 
        }

        Eigen::Vector3d getPointPosition()
        {
            std::unique_lock<std::mutex> lock(mappoint_mutex);
            return poinst_pos; 
        }

        /**
         * @brief add observation of a point 
         * @param feature ptr to feature to be add 
         */
        void addObservation(std::shared_ptr<Feature> feature)
        {   
            std::unique_lock<std::mutex> lock(mappoint_mutex);
            observations.emplace_back(feature); 
            observed_times++; 
        }

        std::list<std::weak_ptr<Feature>> getObservations()
        {
            std::unique_lock<std::mutex> lock(mappoint_mutex);
            return observations; 
        }

        /**
         * @brief remove observation of a point 
         * @param feature ptr to feature to be removed 
         */
        void removeObservation(std::shared_ptr<Feature> feature)
        {
            std::unique_lock<std::mutex> lock(mappoint_mutex); 
            for (auto iter = observations.begin(); iter != observations.end(); iter++) 
            {
                if (iter->lock() == feature) 
                {
                    observations.erase(iter);
                    feature->map_point.reset();
                    observed_times--;
                    break;
                }
            } 
        }
    
    public:
        bool is_outlier = false;
        unsigned int id = 0; ///< map point id number 
        unsigned int observed_times = 0; ///< number of times that point was observed by camera 
    
    private: 

        Eigen::Vector3d poinst_pos = Eigen::Vector3d::Zero(); ///< 3D position of point in space (x,y,z)

        std::list<std::weak_ptr<Feature>> observations; ///< List of features - observation of point by camera

        std::mutex mappoint_mutex; 


    }; 
} //! end of namespace 