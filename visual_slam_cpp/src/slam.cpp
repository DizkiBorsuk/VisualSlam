/**
 * @file slam.cpp
 * @author mrostocki 
 * @brief 
 * @version 0.1
 * @date 2024-03-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "mrVSLAM/slam.hpp" 
#include "mrVSLAM/tools.hpp"
#include "mrVSLAM/frame.hpp" 
#include <boost/config.hpp>
#include <boost/format.hpp>

namespace mrVSLAM
{
    /**
     * @brief Construct a new SLAM::SLAM object - //! to implement
     * 
     * @param config_path - path to configuration file 
     */
    SLAM::SLAM(std::string config_path)
    {
        
    }

    SLAM::SLAM(std::string path_to_dataset, SLAM_TYPE type_of_algorithm, bool loop_closer)
    {
        tracking_type = type_of_algorithm; 
        use_loop_closing = loop_closer; 
        dataset_path = path_to_dataset; 
    }
    
    void SLAM::setSlamParameters(unsigned int num_of_tracked_points, DetectorType type_of_detector, float resize)
    {
        number_of_points = num_of_tracked_points; 
        detector_type = type_of_detector; 
        img_size_opt = resize; 
    }

    void SLAM::initSLAM()
    {
        std::cout << "start of slam initialization \n"; 
        //read data
        dataset = std::shared_ptr<KITTI_Dataset>(new KITTI_Dataset(dataset_path)); 
        dataset->readCalibData(); 

        // create map, local mapping and visualizer objects 
        local_mapping = std::shared_ptr<LocalMapping>(new LocalMapping); 
        map = std::shared_ptr<Map>(new Map); 
        visualizer = std::shared_ptr<Visualizer>(new Visualizer(false));

        // create camera objects
        left_camera = std::shared_ptr<Camera>(new Camera(dataset->P0, img_size_opt));
        right_camera = std::shared_ptr<Camera>(new Camera(dataset->P1, img_size_opt));
        
        // create and set loop closer object if used  
        if(use_loop_closing)
        {
            loop_closer = std::shared_ptr<LoopCloser>(new LoopCloser(vocab_path)); 
            loop_closer->setLoopCloser(map, local_mapping, left_camera, right_camera); 
        }

        switch(tracking_type)
        {
            case SLAM_TYPE::STEREO: 
                stereo_tracking = std::shared_ptr<StereoTracking>(new StereoTracking(DetectorType::GFTT, use_loop_closing, number_of_points)); 
                stereo_tracking->setTracking(map, local_mapping, loop_closer, visualizer, left_camera, right_camera); 
                break; 
            case SLAM_TYPE::MONO:
                mono_tracking = std::shared_ptr<MonoTracking>(new MonoTracking(DetectorType::GFTT, use_loop_closing, number_of_points)); 
                mono_tracking->setTracking(map, local_mapping, loop_closer, visualizer, left_camera); 
                break; 
        }

        local_mapping->setLocalMapping(map, loop_closer, left_camera, right_camera); 
        visualizer->setupVisualizer(map); 
    }

    void SLAM::runSLAM()
    {
        std::cout << "Start of slam execution \n"; 
        // main thread loop 
        while(true)
        {
            try
            {
                //? maybe i should do everything in loop 
                if(createNewFrameAndTrack() == false) 
                {
                    break; 
                }
            }
            catch(const std::exception &e)
            {
                std::cout << "cached critical error, ending slam \n"; 
                std::cout << e.what() << std::endl; 
                break; 
            }
        }
        // close all modules and end threads
        if(loop_closer)
            loop_closer->stop(); 
            
        local_mapping->stop(); 
        visualizer->close(); 
    }

    bool SLAM::createNewFrameAndTrack()
    {
        static unsigned int current_image_index = 0; 
        bool status = true; 

        boost::format fmt("%s/image_%d/%06d.png");
        cv::Mat image_left, image_right, img_left_resized, img_right_resized;

        // 
        auto beginT = std::chrono::steady_clock::now();
        // read imgs 
        image_left = cv::imread((fmt % dataset_path % 0 % current_image_index).str(), cv::IMREAD_UNCHANGED);
        image_right = cv::imread((fmt % dataset_path % 1 % current_image_index).str(), cv::IMREAD_UNCHANGED);

        if(image_left.data == nullptr || image_right.data == nullptr)
        {
            throw std::runtime_error("img data is missing!! \n"); 
        }

        cv::resize(image_left, img_left_resized, cv::Size(), img_size_opt, img_size_opt, cv::INTER_NEAREST);
        cv::resize(image_right, img_right_resized, cv::Size(), img_size_opt, img_size_opt, cv::INTER_NEAREST);

        auto new_frame = std::shared_ptr<Frame> (new Frame(current_image_index, img_left_resized, img_right_resized)); 
        current_image_index++; 

        if(new_frame == nullptr)
        {
            throw std::runtime_error("frame object wasn't created \n"); 
        }
        
        switch (tracking_type)
        {
        case SLAM_TYPE::STEREO:
            status = stereo_tracking->addNewFrame(new_frame); 
            break;
        case SLAM_TYPE::MONO: 
            mono_tracking->addNewFrame(new_frame); 
            break;        
        default:
            break;
        }

        auto endT = std::chrono::steady_clock::now();
        auto elapsedT = std::chrono::duration_cast<std::chrono::milliseconds>(endT - beginT);
        std::cout << "loop time is = " << elapsedT.count() << "ms \n";  
        loop_times.emplace_back(elapsedT.count()); 
        all_frames.emplace_back(new_frame); 
        
        return status; 
    }

    void SLAM::outputSlamResult()
    {
        // dataset->getGTposes(); 
        
        saveResults(); 
    }

    void SLAM::saveResults()
    {
        std::ofstream outputFile;
        outputFile.open("results.csv", std::ios_base::app);

        if(!outputFile)
        {
            std::cerr << "Error:: Couldn't open result output file" << std::endl; 
            std::exit(1); 
        }

        outputFile << "NEW TEST: \n Tracking type, " << to_underlying(tracking_type)  << "\n"; 
        outputFile << "Detector type, " << to_underlying(detector_type) << "\n"; 
        outputFile << "Number of detected features, " << 1 << "\n";  
        outputFile << "Loop closing?, " << use_loop_closing << "\n"; 
        outputFile << "\n"; 
        
        outputFile << "Number of generated keyframes," << map->getNumberOfKeyframes() << "\n";  
        outputFile << "total mean error: ," << 1 << "," << 1 << "\n"; 
        outputFile << "mean error:, x , y , z \n"; 
        outputFile << "," << 1 << "," << 1 << "," << 1 << "\n"; 
        outputFile << "max error:, x, y, z \n"; 
        outputFile << "," << 1 << "," << 1 << "," << 1 << "\n"; 

        outputFile << "END OF TEST RESULTS \n"; 
        outputFile << "\n"; 

        outputFile.close(); 
    }

    void saveTrajectoryAndMap()
    {
        
    }

} //! end of namespace 