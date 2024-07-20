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
#include "mrVSLAM/common_includes.hpp"
#include "mrVSLAM/read_dataset.hpp"
#include "mrVSLAM/tools.hpp"
#include "mrVSLAM/frame.hpp"
#include "mrVSLAM/mappoint.hpp"
#include <boost/config.hpp>
#include <boost/format.hpp>
#include <memory>
#include <netcdf>

namespace mrVSLAM
{
    //!to be implemented
    SLAM::SLAM(std::string config_path)
    {

    }

    SLAM::SLAM(std::string path_to_dataset,DatasetVersion dataset_to_run, SLAM_TYPE type_of_algorithm, bool loop_closer)
    {
        tracking_type = type_of_algorithm;
        use_loop_closing = loop_closer;
        dataset_path = path_to_dataset;
        dataset_type = dataset_to_run;
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
        switch (dataset_type) {
            case DatasetVersion::KITTI:
                dataset = std::make_unique<KITTI_Dataset>(dataset_path);
                img_distorted = false; 
                break;
            case DatasetVersion::EUROC:
                dataset = std::make_unique<EuRoC_Dataset>(dataset_path);
                img_distorted = true; 
                break;
            case DatasetVersion::TUM:
                // dataset = std::make_unique<KITTI_Dataset>(dataset_path);
                img_distorted = true; 
                break;
        }

        dataset->readCalibData();
        auto left_cam_dist_coeffs = dataset->returnLeftCamDistCoeffs();
        auto right_cam_dist_coeffs = dataset->returnLeftCamDistCoeffs();
        auto left_cam_matrix = dataset->returnP0();
        auto right_cam_matrix = dataset->returnP1();
        auto img_size = dataset->returnDatasetImgSize(); 

        dataset->showPmatricies();

        // create camera objects
        left_camera = std::make_shared<Camera>(left_cam_matrix, left_cam_dist_coeffs,img_size, img_size_opt);
        right_camera = std::make_shared<Camera>(right_cam_matrix, right_cam_dist_coeffs,img_size, img_size_opt);

        if(img_distorted == true) {
            stereo_set = std::make_unique<StereoCameraSet>(left_camera, right_camera);
        } 

        // create map, local mapping and visualizer objects
        local_mapping = std::make_shared<LocalMapping>();
        map = std::make_shared<Map>();
        visualizer = std::make_shared<Visualizer>(false, this->show_cam_img);

        double similarity_score = 0;
        switch (detector_type)
        {
        case DetectorType::ORB:
            similarity_score = 0.03;
            break;
        case DetectorType::GFTT:
             similarity_score = 0.055;
             break;
        default:
            break;
        }

        // create and set loop closer object if used
        if(use_loop_closing) {
            loop_closer = std::make_shared<LoopCloser>(vocab_path, true, similarity_score);
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

        std::filesystem::path left_img_path;  
        std::filesystem::path right_img_path; 
        
        switch (dataset_type) {
            case DatasetVersion::KITTI:
                left_img_path = (dataset_path + "/image_0"); 
                right_img_path = (dataset_path + "/image_1"); 
                break;
            case DatasetVersion::EUROC:
                left_img_path = dataset_path + "/cam0/data"; 
                right_img_path = dataset_path + "/cam1/data"; 
                break;
            case DatasetVersion::TUM:
                break;
        }

        std::cout <<"path = "<< left_img_path.generic_string() << "\n"; 

        std::set<std::filesystem::path> left_set; 
        std::set<std::filesystem::path> right_set; 

        for(auto &entry : std::filesystem::directory_iterator(left_img_path)) {
            left_set.insert(entry.path()); 
        }

        for(auto &entry : std::filesystem::directory_iterator(right_img_path)) {
            right_set.insert(entry.path()); 
        }

        std::vector<std::pair<std::filesystem::path, std::filesystem::path>> img_files_names;
    
        auto it1 = left_set.begin(); 
        auto it2 = right_set.begin(); 

        for(size_t i = 0; i < left_set.size(); i++)
        {
            std::pair<std::filesystem::path, std::filesystem::path> path_pair {*it1, *it2}; 
            img_files_names.emplace_back(path_pair);
            it1++; 
            it2++;  
        }
        // main thread loop

        size_t video_offset = 0; 
        for(size_t im = video_offset; im < img_files_names.size(); im++)
        {
            try
            {
                if(createNewFrameAndTrack(img_files_names.at(im).first.generic_string(), img_files_names.at(im).second.generic_string()) == false) {
                    this->vslam_failed = true;
                    break;
                }
            }
            catch(std::exception& e)
            {
                fmt::print(fg(fmt::color::red), "cached critical error, {} \n, ending slam \n", e.what());
                break;
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

    bool SLAM::createNewFrameAndTrack(const std::string left_img_path, const std::string right_img_path)
    {
        static unsigned int current_image_index = 0;
        bool status = true;

        cv::Mat image_left, image_right, corrected_img_left, corrected_img_right;

        //
        auto beginT = std::chrono::steady_clock::now();

        // read imgs
        image_left = cv::imread(left_img_path, cv::IMREAD_UNCHANGED);
        image_right = cv::imread(right_img_path, cv::IMREAD_UNCHANGED);

        std::cout << "img path = " << left_img_path << "\n"; 
        std::cout << "img path = " << right_img_path << "\n"; 


        if(image_left.data == nullptr || image_right.data == nullptr)
        {
            throw std::runtime_error("img data is missing!! \n");
        }

        if(img_distorted && stereo_set)
        {
            stereo_set->rectifyStereoImgs(image_left, image_right); 
        }

        auto new_frame = std::make_shared<Frame> (current_image_index, image_left, image_right);

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

    void SLAM::outputSlamResult(const bool plot)
    {
        if(this->vslam_failed){
            fmt::print("###----------------------### \n");
            fmt::print(fg(fmt::color::red), "no output to save, vslam failed \n");
            return;
        }

        fmt::print("###----------------------### \n");
        fmt::print(fg(fmt::color::green), "create and save slam output \n");

        //* Get all needed trajectories
        dataset->readGTposes();
        auto gt_trajectory = dataset->retrunGTposes();

        std::vector<Eigen::Matrix<double, 3,4>> trajectory;
        std::vector<Eigen::Matrix<double, 3,4>> kf_trajectory;
        std::vector<Eigen::Matrix<double, 3,4>> no_kf_trajectory;
        std::vector<Eigen::Matrix<double, 3,4,1>> gt_kf_trajectory;

        for(auto& frame: all_frames) {
            trajectory.emplace_back(frame->getPose().inverse().matrix3x4());
            if(frame->is_keyframe == false) {
                no_kf_trajectory.emplace_back(frame->getPose().inverse().matrix3x4());
            }
        }

        auto keyframes = map->getAllKeyframes();
        std::map<unsigned int, std::shared_ptr<Frame>> keyframes_in_order(keyframes.begin(), keyframes.end());

        std::vector<int> kf_idx;
        for(auto& [key, frame] : keyframes_in_order) {
            Eigen::Matrix<double,3,4> pose = frame->getPose().inverse().matrix3x4();
            kf_trajectory.emplace_back(pose);
            kf_idx.emplace_back(frame->id);
        }

        for(size_t i = 0; i < kf_idx.size(); i++) {
            gt_kf_trajectory.emplace_back(gt_trajectory.at(kf_idx.at(i)));
        }

        //* Save to netcdf file
        saveTrajectoryToNetCDF(trajectory, kf_trajectory, no_kf_trajectory, gt_kf_trajectory);

        ResultStruct results;
        results.sequence = dataset->getCurrentSequence();
        results.detector = detector_type;
        results.tracking_type = tracking_type;
        results.num_of_features = number_of_points;

        //* calculate errors

        calculate_error(trajectory, gt_trajectory, img_size_opt, results.sequence , results);
        calculate_kf_error(kf_trajectory, gt_kf_trajectory, img_size_opt, results.sequence , results);
        calculate_time(loop_times, results);

        //* save to csv file
        saveResultsToCSV(results);

        //* plot the stuff

        if(plot)
        {
            plotPoses(trajectory, gt_trajectory, 1, "All frames");

            if(this->use_loop_closing)
            {
                plotPoses(kf_trajectory, gt_trajectory, 1, "Only optimized frames");
                plotPoses(no_kf_trajectory, gt_trajectory, 1, "Only nonoptimized frames");

                auto matched_keyframes = map->getAllMatchedKeyframes();
                fmt::print("Loop Closing module found {} matching keyframe pairs \n", matched_keyframes.size());
                plotLoopClosingMatches(matched_keyframes, kf_trajectory, 1);
            }
        }
    }

    void SLAM::saveResultsToCSV(const ResultStruct &results)
    {
        std::ofstream outputFile;
        outputFile.open("results.csv", std::ios_base::app);

        if(!outputFile) {
            std::cerr << "Error:: Couldn't open result output file" << std::endl;
            std::exit(1);
        }

        outputFile << "sequence number: ,"  << results.sequence << "\n" ;
        outputFile << "Tracking type, "     << to_underlying(results.tracking_type)  << "\n";
        outputFile << "Detector type, "     << to_underlying(results.detector) << "\n";
        outputFile << "Loop closing?, "     << use_loop_closing << "\n";
        outputFile << "Number of detected features, "       << results.num_of_features << "\n";
        outputFile << "Number of detected loop closings, "  << map->getAllMatchedKeyframes().size() << "\n";
        outputFile << "Number of generated keyframes,"      << map->getNumberOfKeyframes() << "\n";
        outputFile << "total mean error:,"      << results.mean_error << "\n";
        outputFile << "kf mean error:,"         << results.mean_kf_error << "\n";
        outputFile << "total mean error %:,"    << results.percent_error << "\n";
        outputFile << "kf mean error %:,"       << results.percent_kf_error << "\n";
        outputFile << "mean error x:,"          << results.mean_error_x << "\n";
        outputFile << "mean error y:,"          << results.mean_error_y << "\n";
        outputFile << "mean error z:,"          << results.mean_error_z << "\n";
        outputFile << "mean kf error x:,"       << results.mean_kf_error_x << "\n";
        outputFile << "mean kf error y:,"       << results.mean_kf_error_y << "\n";
        outputFile << "mean kf error z:,"       << results.mean_kf_error_z << "\n";
        outputFile << "max error x:,"           << results.max_error_x << "\n";
        outputFile << "max error y:,"           << results.max_error_y << "\n";
        outputFile << "max error z:,"           << results.max_error_z << "\n";
        outputFile << "mean loop time:, "       << results.mean_time << "\n";
        outputFile << "max loop time:, "        << results.max_time << "\n";
        outputFile << "min loop time:, "        << results.min_time << "\n";
        outputFile.close();
    }

    void SLAM::saveTrajectoryToNetCDF(std::vector<Eigen::Matrix<double, 3,4>>& trajectory,
                                      std::vector<Eigen::Matrix<double, 3,4>>& kf_trajectory,
                                      std::vector<Eigen::Matrix<double, 3,4>>& no_kf_trajectory,
                                      std::vector<Eigen::Matrix<double, 3,4,1>>& gf_kf_trajectory)
    {
        std::string filename = "map_trajectory.nc";
        netCDF::NcFile mapFile(filename, netCDF::NcFile::replace);

        auto mappoints = map->getAllMappoints();

        constexpr int matrix_width  = 4; // num of columns
        constexpr int matrix_height = 3; //num of rows
        constexpr int vector_length = 3;

        int num_of_all_frames       = trajectory.size();
        int num_of_keyframes        = kf_trajectory.size();
        int num_of_non_kf_frames    = no_kf_trajectory.size();
        int num_of_mappoints        = mappoints.size();
        int num_of_loop_closes      = map->getAllMatchedKeyframes().size();

        //*reasign all poses to ncFloat
        float out_all_poses[num_of_all_frames][matrix_height][matrix_width];
        float output_kf_poses[num_of_keyframes][matrix_height][matrix_width];
        float output_gf_kf_poses[num_of_keyframes][matrix_height][matrix_width];
        float output_no_kf_poses[num_of_non_kf_frames][matrix_height][matrix_width];


        for(size_t i = 0; i < trajectory.size(); i++) {
            for (size_t i_r = 0; i_r < matrix_height; i_r++) {
                for (size_t i_c = 0; i_c < matrix_width; i_c++) {
                    out_all_poses[i][i_r][i_c] = trajectory.at(i).coeff(i_r,i_c);
                }
            }
        }

        //* reasign kf poses to ncFloat
        for(size_t i = 0; i < kf_trajectory.size(); i++) {
            for (size_t i_r = 0; i_r < matrix_height; i_r++) {
                for (size_t i_c = 0; i_c < matrix_width; i_c++) {
                    output_kf_poses[i][i_r][i_c] = kf_trajectory.at(i).coeff(i_r,i_c);
                    output_gf_kf_poses[i][i_r][i_c] = gf_kf_trajectory.at(i).coeff(i_r,i_c);
                }
            }
        }


        for(size_t i = 0; i < no_kf_trajectory.size(); i++) {
            for (size_t i_r = 0; i_r < matrix_height; i_r++) {
                for (size_t i_c = 0; i_c < matrix_width; i_c++) {
                    output_no_kf_poses[i][i_r][i_c] = no_kf_trajectory.at(i).coeff(i_r,i_c);
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
        float out_matched_kf_ids [num_of_loop_closes][2];
        auto mathched_kfs = map->getAllMatchedKeyframes();
        for(int i = 0; i < num_of_loop_closes; i++) {
            out_matched_kf_ids[i][0] = mathched_kfs.at(i).first->kf_id;
            out_matched_kf_ids[i][1] = mathched_kfs.at(i).second->kf_id;
        }

        //create dimmensions
        auto matrix_width_dim       = mapFile.addDim("matrix_width"  , matrix_width);
        auto matrix_height_dim      = mapFile.addDim("matrix_height" , matrix_height);
        auto kf_poses_num_dim       = mapFile.addDim("kf_poses_num", num_of_keyframes); //TODO maybe change to poses
        auto no_kf_poses_num_dim    = mapFile.addDim("no_kf_poses_num", num_of_non_kf_frames);
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
            loop_closing_var = mapFile.addVar("matched_kf", netCDF::ncFloat, {num_of_loop_closes, pair_dim});
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
                loop_closing_var.putVar(out_matched_kf_ids);
            }

        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        fmt::print("saved map");
    }

} //! end of namespace
