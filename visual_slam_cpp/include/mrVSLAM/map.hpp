/**
 * @file map.hpp
 * @author mrostocki
 * @brief map class that stores all keyframes and map points in hashtables 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once 
#include "mrVSLAM/common_includes.hpp" 

namespace mrVSLAM
{
    class MapPoint; 
    class KeyFrame; 
    class Feature; 
    class Frame; 
    
    /**
     * @brief map class that stores all keyframes and map points in hashtables 
     */
    class Map
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 
        typedef std::unordered_map<unsigned int, std::shared_ptr<Frame>> KeyframesType; 
        typedef std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> LandmarksType; 
        std::mutex mapUpdate_mutex; 

    public: 
        Map() = default; 

        void insertNewKeyframe(std::shared_ptr<Frame> frame); 
        void insertNewMappoint(std::shared_ptr<MapPoint> map_point); 

        /**
         * @brief Get the Kyeframe object
         * @details returns keyframe based on passed id 
         * @param keyframe_id 
         * @return std::shared_ptr<Frame> 
         */
        std::shared_ptr<Frame> getKyeframeById(unsigned int keyframe_id); 

        void cleanMap(); 


        LandmarksType getAllMappoints()
        {
            std::unique_lock<std::mutex> lock(map_mutex);
            return allMappointsDictionary; 
        }
        LandmarksType getActiveMappoints()
        {
            std::unique_lock<std::mutex> lock(map_mutex);
            return activeMappointsDictionary; 
        }
        KeyframesType getAllKeyframes()
        {
            std::unique_lock<std::mutex> lock(map_mutex);
            return allKeyframesDictionary; 
        }
        KeyframesType getActiveKeyframes()
        {
            std::unique_lock<std::mutex> lock(map_mutex);
            return activeKeyframesDictionary; 
        }

        int getNumberOfKeyframes()
        {
            return allKeyframesDictionary.size(); 
        }

        int getNumberOfMapPoints()
        {
            return allMappointsDictionary.size(); 
        }

    private: 
        void removeOldKeyframe(); 

    private: 
        std::mutex map_mutex; 
        std::mutex outliers_mutex; 

        std::shared_ptr<Frame> current_keyframe = nullptr; 
        std::list<unsigned int> outlier_mappoints_list; 
        
        LandmarksType allMappointsDictionary; ///< hash map that contains all mappoint objects 
        LandmarksType activeMappointsDictionary; ///< hash map that contains only map points in local map 
        KeyframesType allKeyframesDictionary; ///< hash map that contains all keyframe objects
        KeyframesType activeKeyframesDictionary; ///< hash map that contains only keyframes of local map

        static constexpr int num_of_active_keyframes = 10; ///< size of local map (number of stored keyframes)
    }; 

} //! end of namespace 