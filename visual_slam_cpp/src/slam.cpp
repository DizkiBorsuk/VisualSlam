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
#include "mrVSLAM/mappoint.hpp"
#include <boost/config.hpp>
#include <boost/format.hpp>
#include <netcdf>

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
    
    void SLAM::setSlamParameters(DetectorType type_of_detector, unsigned int num_of_tracked_points, unsigned int min_tracking_points, 
                                 float resize, bool show_cam_img, std::string vocab_path )
    {
        this->number_of_points = num_of_tracked_points; 
        this->detector_type = type_of_detector; 
        this->img_size_opt = resize; 
        this->show_cam_img = show_cam_img; 
        this->vocab_path = vocab_path; 
        this->min_tracking_points = min_tracking_points; 
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
            loop_closer = std::make_shared<LoopCloser>(vocab_path, true); 
            loop_closer->setLoopCloser(map, local_mapping, left_camera, right_camera); 
        }

        switch(tracking_type)
        {
            case SLAM_TYPE::STEREO: 
                stereo_tracking = std::make_shared<StereoTracking>(this->detector_type, this->number_of_points, this->min_tracking_points, this->use_loop_closing); 
                stereo_tracking->setTracking(map, local_mapping, loop_closer, visualizer, left_camera, right_camera); 
                break; 
            case SLAM_TYPE::MONO:
                mono_tracking = std::make_shared<MonoTracking>(this->detector_type, use_loop_closing, number_of_points); 
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
                    this->vslam_failed = true; 
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
        if(this->vslam_failed){
            fmt::print("###----------------------### \n");
            fmt::print(fg(fmt::color::red), "no output to save, vslam failed \n"); 
            return; 
        }

        fmt::print("###----------------------### \n");
        fmt::print(fg(fmt::color::green), "create and save slam output \n"); 

        ResultStruct results; 

        dataset->getGTposes();
        std::vector<Eigen::Matrix<double, 3,4>> trajectory;  
        std::vector<Eigen::Matrix<double, 3,4>> kf_trajectory;
        std::vector<Eigen::Matrix<double, 3,4>> no_kf_trajectory;  

        // auto keyframes = map->getAllKeyframes();  
        for(auto& frame: all_frames)
        {
            trajectory.emplace_back(frame->getPose().inverse().matrix3x4()); 
            if(frame->is_keyframe) {
                kf_trajectory.emplace_back(frame->getPose().inverse().matrix3x4()); 
            } else {
                no_kf_trajectory.emplace_back(frame->getPose().inverse().matrix3x4()); 
            }
        }

        results.sequence = dataset->getCurrentSequence(); 
        results.detector = detector_type; 
        results.tracking_type = tracking_type; 
        results.num_of_features = number_of_points;

        if(plot)
        {
            plotPoses(trajectory, dataset->ground_truth_poses, 1, "All frames"); 
            plotPoses(kf_trajectory, dataset->ground_truth_poses, 1, "Only optimized frames"); 
            plotPoses(no_kf_trajectory, dataset->ground_truth_poses, 1, "Only nonoptimized frames"); 
            
            if(this->use_loop_closing)
            {
                auto matched_keyframes = map->getAllMatchedKeyframes(); 
                fmt::print("Loop Closing module found {} matching keyframe pairs \n", matched_keyframes.size()); 
                plotLoopClosingMatches(matched_keyframes, no_kf_trajectory, 1); 
                plotLoopClosingMatches(matched_keyframes, kf_trajectory, 1); 
            }

        }

        calculate_error(trajectory, dataset->ground_truth_poses, img_size_opt, 6 , results); 
        calculate_time(loop_times, results);
        saveResults(results); 
        saveTrajectoryAndMap(); 
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

        outputFile << "sequence number: ,"  << results.sequence << "\n" ;
        outputFile << "Tracking type, "     << to_underlying(results.tracking_type)  << "\n"; 
        outputFile << "Detector type, "     << to_underlying(results.detector) << "\n"; 
        outputFile << "Loop closing?, "     << use_loop_closing << "\n"; 
        outputFile << "Number of detected features, " << results.num_of_features << "\n";  
        outputFile << "Number of detected loop closings, " << map->getAllMatchedKeyframes().size(); 
        outputFile << "Number of generated keyframes," << map->getNumberOfKeyframes() << "\n";  
        outputFile << "total mean error:,"      << results.mean_error << "\n";  
        outputFile << "total mean error %:,"    << results.percent_error << "\n"; 
        outputFile << "mean error x:,"          << results.mean_error_x << "\n"; 
        outputFile << "mean error y:,"          << results.mean_error_y << "\n";
        outputFile << "mean error z:,"          << results.mean_error_z << "\n";
        outputFile << "max error x:,"           << results.max_error_x << "\n";  
        outputFile << "max error y:,"           << results.max_error_y << "\n"; 
        outputFile << "max error y:,"           << results.max_error_z << "\n"; 
        outputFile << "mean loop time:, "       << results.mean_time << "\n"; 
        outputFile << "max loop time:, "        << results.max_time << "\n";
        outputFile << "min loop time:, "        << results.min_time << "\n"; 
        outputFile.close(); 
    }

    void SLAM::saveTrajectoryAndMap()
    {
        std::string filename = "map_trajectory.nc"; 
        netCDF::NcFile mapFile(filename, netCDF::NcFile::replace); 
        
        auto keyframes = map->getAllKeyframes(); 
        auto mappoints = map->getAllMappoints(); 

        constexpr int matrix_width = 4; // num of columns 
        constexpr int matrix_height = 3; //num of rows 
        constexpr int vector_length = 3; 

        int num_of_keyframes = keyframes.size(); 
        int num_of_mappoints = mappoints.size(); 
        int num_of_all_frames = all_frames.size(); 
        int num_of_loop_closes = map->getAllMatchedKeyframes().size(); 

        //*reasign all poses to ncFloat
        float out_all_poses[num_of_all_frames][matrix_height][matrix_width];
        for(size_t i = 0; i < all_frames.size(); i++) {
            Eigen::Matrix<double,3,4> pose = all_frames.at(i)->getPose().inverse().matrix3x4(); 
            for (size_t i_r = 0; i_r < matrix_height; i_r++) {
                for (size_t i_c = 0; i_c < matrix_width; i_c++) {
                    out_all_poses[i][i_r][i_c] = pose.coeff(i_r,i_c); 
                }    
            }
        }
        
        //* reasign kf poses to ncFloat  
        std::map<unsigned int, std::shared_ptr<Frame>> keyframes_in_order(keyframes.begin(), keyframes.end()); 
        float output_kf_poses[num_of_keyframes][matrix_height][matrix_width]; 

        for(auto& [key, frame] : keyframes_in_order) {
            Eigen::Matrix<double,3,4> pose = frame->getPose().inverse().matrix3x4(); 
            for (size_t i_r = 0; i_r < matrix_height; i_r++) {
                for (size_t i_c = 0; i_c < matrix_width; i_c++) {
                    output_kf_poses[key][i_r][i_c] = pose.coeff(i_r,i_c); 
                }    
            }
        }

        //* retrieve and reasign ground truth poses to ncFloat 
        float output_gf_kf_poses[num_of_keyframes][matrix_height][matrix_width]; 
        std::vector<int> idx; 
        for(auto& [key, frame] : keyframes_in_order) {
            idx.emplace_back(frame->id); 
        }

        for(size_t i = 0; i < idx.size(); i++) {
            for (size_t i_r = 0; i_r < matrix_height; i_r++) {
                for (size_t i_c = 0; i_c < matrix_width; i_c++) {
                    output_gf_kf_poses[i][i_r][i_c] = dataset->ground_truth_poses.at(i).coeff(i_r,i_c); 
                }
            }
        }

        //* retrieve and save no optimized kf 
        std::vector<Eigen::Matrix<double,3,4>> no_kf_poses; 
        for(auto& frame : this->all_frames)
        {
            if(!frame->is_keyframe) {
                no_kf_poses.emplace_back(frame->getPose().inverse().matrix3x4()); 
            }
        }

        float output_no_kf_poses[no_kf_poses.size()][matrix_height][matrix_width]; 
        for(size_t i = 0; i < no_kf_poses.size(); i++) {
            for (size_t i_r = 0; i_r < matrix_height; i_r++) {
                for (size_t i_c = 0; i_c < matrix_width; i_c++) {
                    output_no_kf_poses[i][i_r][i_c] = no_kf_poses.at(i).coeff(i_r,i_c); 
                }
            }
        }

        //* reasign mappoint positions to ncFloat
        float output_mappoints[num_of_mappoints][vector_length]; 

        for(auto& [key, mappoint] : mappoints) {
            Eigen::Vector3d position = mappoint->getPointPosition(); 
            output_mappoints[key][0] = position.coeff(0); 
            output_mappoints[key][1] = position.coeff(1); 
            output_mappoints[key][2] = position.coeff(2); 
        }

        //* reasign loop times to ncFloat
        float output_loop_times[num_of_all_frames]; 
        for(int i = 0; i < num_of_all_frames; i++){
            output_loop_times[i] = this->loop_times.at(i); 
        }

        //* reasign matched kf keyframes id's
        float out_matche_kf_ids [num_of_loop_closes][2];
        auto mathched_kfs = map->getAllMatchedKeyframes(); 
        for(int i = 0; i < num_of_loop_closes; i++) {
            out_matche_kf_ids[i][0] = mathched_kfs.at(i).first->kf_id; 
            out_matche_kf_ids[i][1] = mathched_kfs.at(i).second->kf_id; 
        } 

        //create dimmensions 
        auto matrix_width_dim       = mapFile.addDim("matrix_width"  , matrix_width); 
        auto matrix_height_dim      = mapFile.addDim("matrix_height" , matrix_height); 
        auto kf_poses_num_dim       = mapFile.addDim("kf_poses_num", num_of_keyframes); //TODO maybe change to poses 
        auto no_kf_poses_num_dim    = mapFile.addDim("no_kf_poses_num", no_kf_poses.size()); 
        auto vector_length_dim      = mapFile.addDim("vector_length", vector_length); 
        auto mappoints_num_dim      = mapFile.addDim("mappoints_num", num_of_mappoints); 
        auto loop_times_dim         = mapFile.addDim("loop_time_num", num_of_all_frames);    
        auto loop_closes_dim        = mapFile.addDim("num_of_loop_closing", num_of_loop_closes); 
        auto pair_dim               = mapFile.addDim("matched_frames_num", 2); 

        //create variables
        auto all_poses_var      = mapFile.addVar("all_poses", netCDF::ncFloat, {loop_times_dim, matrix_height_dim, matrix_width_dim}); 
        auto kf_poses_var       = mapFile.addVar("kf_poses", netCDF::ncFloat ,{kf_poses_num_dim, matrix_height_dim, matrix_width_dim}); 
        auto gt_kf_poses_var    = mapFile.addVar("ground_truth_kf_poses", netCDF::ncFloat, {kf_poses_num_dim, matrix_height_dim, matrix_width_dim}); 
        auto no_kf_poses_var    = mapFile.addVar("no_kf_poses", netCDF::ncFloat, {no_kf_poses_num_dim, matrix_height_dim, matrix_width_dim}); 
        auto mappoints_var      = mapFile.addVar("mappoints_set", netCDF::ncFloat, {mappoints_num_dim, vector_length_dim}); 
        auto loop_times_var     = mapFile.addVar("loop_times", netCDF::ncFloat, {loop_times_dim}); 
        netCDF::NcVar loop_closing_var; 
        if(this->use_loop_closing) {
            loop_closing_var = mapFile.addVar("loop_times", netCDF::ncFloat, {num_of_loop_closes, pair_dim});
        }
        
        //assign inputs to variables 
        try
        {
            all_poses_var.putVar(out_all_poses); 
            kf_poses_var.putVar(output_kf_poses); 
            gt_kf_poses_var.putVar(output_gf_kf_poses); 
            no_kf_poses_var.putVar(output_no_kf_poses); 
            mappoints_var.putVar(output_mappoints); 
            loop_times_var.putVar(output_loop_times); 
            
            if(this->use_loop_closing) {
                loop_closing_var.putVar(out_matche_kf_ids);
            }
        
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        fmt::print("saved map"); 
    }

} //! end of namespace 