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

        /**
         * @brief Get the All Mappoints objects stored in a hash map 
         * @return LandmarksType - hash map of stored mappoints 
         */
        LandmarksType getAllMappoints()
        {
            std::unique_lock<std::mutex> lock(map_mutex);
            return allMappointsDictionary; 
        }

        /**
         * @brief Get the Active Mappoints objects (local map of last 10 frames) stored in a hash map 
         * @return LandmarksType - hash map of stored mappoints 
         */
        LandmarksType getActiveMappoints()
        {
            std::unique_lock<std::mutex> lock(map_mutex);
            return activeMappointsDictionary; 
        }

        /**
         * @brief Get the All Keyframes objects stored in a hash map 
         * @return KeyframesType - hash map of all stored frame objects (keyframes)
         */
        KeyframesType getAllKeyframes()
        {
            std::unique_lock<std::mutex> lock(map_mutex);
            return allKeyframesDictionary; 
        }

        /**
         * @brief Get the Active Keyframes objects (local map of last 10 frames) stored in a hash map 
         * @return KeyframesType 
         */
        KeyframesType getActiveKeyframes()
        {
            std::unique_lock<std::mutex> lock(map_mutex);
            return activeKeyframesDictionary; 
        }

        /**
         * @brief Get the Number Of Keyframe object stored in map
         * @return int 
         */
        int getNumberOfKeyframes()
        {
            return allKeyframesDictionary.size(); 
        }

        /**
         * @brief Get the Number Of Map Point objects stored in map 
         * @return int 
         */
        int getNumberOfMapPoints()
        {
            return allMappointsDictionary.size(); 
        }

        void addMatchedKeyframes(std::shared_ptr<Frame> older_kf, std::shared_ptr<Frame> current_kf)
        {
            std::unique_lock<std::mutex> lock(map_mutex);
            matched_keyframes.emplace_back(std::make_pair(older_kf, current_kf)); 
        }

        std::vector<std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>>> getAllMatchedKeyframes()
        {
            std::unique_lock<std::mutex> lock(map_mutex);
            return matched_keyframes; 
        }

        void removeMapPoint(std::shared_ptr<MapPoint> mappoint); 
    private: 
        /**
         * @brief remove old (older than 10) keyframe and associated with it mappoints 
         */
        void removeOldKeyframeAndMappoints(); 

    private: 
        std::mutex map_mutex; 
        std::mutex outliers_mutex; 

        std::shared_ptr<Frame> current_keyframe = nullptr; 
        std::list<unsigned int> outlier_mappoints_list; 
        
        LandmarksType allMappointsDictionary; ///< hash map that contains all mappoint objects 
        LandmarksType activeMappointsDictionary; ///< hash map that contains only map points in local map 
        KeyframesType allKeyframesDictionary; ///< hash map that contains all keyframe objects
        KeyframesType activeKeyframesDictionary; ///< hash map that contains only keyframes of local map

        std::vector< std::pair< std::shared_ptr<Frame>, std::shared_ptr<Frame> >> matched_keyframes; 

        static constexpr int num_of_active_keyframes = 10; ///< size of local map (number of stored keyframes)
    }; 

} //! end of namespace 