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
    //!to be implemented 
    SLAM::SLAM(std::string config_path)
    {
        
    }

    SLAM::SLAM(std::string path_to_dataset, SLAM_TYPE type_of_algorithm, bool loop_closer)
    {
        tracking_type = type_of_algorithm; 
        use_loop_closing = loop_closer; 
        dataset_path = path_to_dataset; 
        fmt::print("SLAM object created \n");
        fmt::print("###----------------------### \n"); 
    }
    
    void SLAM::setSlamParameters(DetectorType type_of_detector, unsigned int num_of_tracked_points, float resize, 
                                bool show_cam_img, std::string vocab_path)
    {
        this->number_of_points = num_of_tracked_points; 
        this->detector_type = type_of_detector; 
        this->img_size_opt = resize; 
        this->show_cam_img = show_cam_img; 
        this->vocab_path = vocab_path; 
    }

    void SLAM::initSLAM()
    {
        fmt::print(fg(fmt::color::green), "start of slam initialization \n"); 
        //read data
        dataset = std::make_shared<KITTI_Dataset>(dataset_path); 
        dataset->readCalibData(); 

        // create map, local mapping and visualizer objects 
        local_mapping = std::make_shared<LocalMapping>(); 
        map = std::make_shared<Map>(); 
        visualizer = std::make_shared<Visualizer>(false, this->show_cam_img);

        // create camera objects
        left_camera = std::make_shared<Camera>(dataset->P0, img_size_opt);
        right_camera = std::make_shared<Camera>(dataset->P1, img_size_opt);
        
        // create and set loop closer object if used  
        if(use_loop_closing) {
            loop_closer = std::make_shared<LoopCloser>(vocab_path); 
            loop_closer->setLoopCloser(map, local_mapping, left_camera, right_camera); 
        }

        switch(tracking_type)
        {
            case SLAM_TYPE::STEREO: 
                stereo_tracking = std::make_shared<StereoTracking>(DetectorType::GFTT, use_loop_closing, number_of_points); 
                stereo_tracking->setTracking(map, local_mapping, loop_closer, visualizer, left_camera, right_camera); 
                break; 
            case SLAM_TYPE::MONO:
                mono_tracking = std::make_shared<MonoTracking>(DetectorType::GFTT, use_loop_closing, number_of_points); 
                mono_tracking->setTracking(map, local_mapping, loop_closer, visualizer, left_camera); 
                break; 
        }

        local_mapping->setLocalMapping(map, left_camera, right_camera); 
        visualizer->setupVisualizer(map); 

        fmt::print(fg(fmt::color::green), "end of slam initialization \n"); 
        fmt::print("###----------------------### \n");
    }

    void SLAM::runSLAM()
    {
        fmt::print(fg(fmt::color::green), "start of slam execution \n"); 
        // main thread loop 
        while(true)
        {
            try
            {
                //? maybe i should do everything in loop 
                if(createNewFrameAndTrack() == false) {
                    break; 
                }
            }
            catch(...)
            {
                fmt::print(fg(fmt::color::red), "cached critical error, ending slam \n"); 
                break; 
            }
        }

        // close all modules and end threads
        if(loop_closer)
            loop_closer->stop(); 
            
        local_mapping->stop(); 
        visualizer->close(); 

        fmt::print(fg(fmt::color::green), "end of slam algorithm execution \n"); 
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

        auto new_frame = std::make_shared<Frame> (current_image_index, img_left_resized, img_right_resized); 
        
        if(new_frame == nullptr) {
            throw std::runtime_error("frame object wasn't created \n"); 
        } else {
            fmt::print(fg(fmt::color::blue), "created new frame with id:{} \n", current_image_index); 
            current_image_index++; 
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
        fmt::print(fg(fmt::color::yellow), "main loop time =  {} ms \n", elapsedT.count()); 
        loop_times.emplace_back(elapsedT.count()); 
        all_frames.emplace_back(new_frame); 
        
        return status; 
    }

    void SLAM:: outputSlamResult(const bool plot)
    {
        fmt::print("###----------------------### \n");
        fmt::print(fg(fmt::color::green), "create and save slam output \n"); 

        ResultStruct results; 

        dataset->getGTposes();
        // std::vector<Eigen::Matrix<double, 3,4>>  kf_trajectory;
        // auto keyframes = map->getAllKeyframes();  
        for(size_t i = 0; i < all_frames.size(); i++)
        {
            trajectory.emplace_back(all_frames.at(i)->getPose().inverse().matrix3x4()); 
        }


        results.sequence = dataset->getCurrentSequence(); 
        results.detector = detector_type; 
        results.tracking_type = tracking_type; 
        results.num_of_features = number_of_points;

        if(plot)
        {
            plotPoses(trajectory, dataset->ground_truth_poses, 1); 
            
            if(this->use_loop_closing)
            {
                auto matched_keyframes = map->getAllMatchedKeyframes(); 
                fmt::print("Loop Closing module found {} matching keyframe pairs", matched_keyframes.size()); 
                plotLoopClosingMatches(matched_keyframes, trajectory, 1); 
            }

        }

        calculate_error(trajectory, dataset->ground_truth_poses, img_size_opt, 6 , results); 
        calculate_time(loop_times, results);
        saveResults(results); 
    }

    void SLAM::saveResults(const ResultStruct &results)
    {
        std::ofstream outputFile;
        outputFile.open("results.csv", std::ios_base::app);

        if(!outputFile)
        {
            std::cerr << "Error:: Couldn't open result output file" << std::endl; 
            std::exit(1); 
        }

        outputFile << "NEW TEST:, sequence number: ," << results.sequence << "\n" ;
        outputFile << "Tracking type, " << to_underlying(results.tracking_type)  << "\n"; 
        outputFile << "Detector type, " << to_underlying(results.detector) << "\n"; 
        outputFile << "Number of detected features, " << results.num_of_features << "\n";  
        outputFile << "Loop closing?, " << use_loop_closing << "\n"; 
        outputFile << "\n"; 
        
        outputFile << "Number of generated keyframes," << map->getNumberOfKeyframes() << "\n";  
        outputFile << "total mean error: ," << results.mean_error << "," << results.percent_error << "\n"; 
        outputFile << "mean error:, x , y , z \n"; 
        outputFile << "," << results.mean_error_x << "," << results.mean_error_y << "," << results.mean_error_z << "\n"; 
        outputFile << "max error:, x, y, z \n"; 
        outputFile << "," << results.max_error_x << "," << results.max_error_y << "," << results.max_error_z << "\n"; 
        outputFile << "mean loop time: , " << results.mean_time << "\n"; 
        outputFile << "max loop time: , " << results.max_time << "\n";
        outputFile << "min loop time: , " << results.min_time << "\n"; 


        outputFile << "END OF TEST RESULTS \n"; 
        outputFile << "\n"; 

        outputFile.close(); 
    }

    void saveTrajectoryAndMap()
    {
        
    }

} //! end of namespace 