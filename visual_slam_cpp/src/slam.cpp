
#include "myslam/slam.hpp"
#include <boost/config.hpp>
#include <boost/format.hpp>
#include "myslam/stereo_tracking_matching.hpp"
#include "myslam/mono_tracking.hpp"


namespace myslam 
{
    SLAM::SLAM(std::string &config_path, slamType type_of_algorithm, bool loop_closer, float resize)
        : dataset_path(config_path),algorithm_type(type_of_algorithm),
        use_loop_closing(loop_closer), img_size_opt(resize)
    {

    }

    void SLAM::Init() 
    {
        // read data
        dataset = std::shared_ptr<KITTI_Dataset>(new KITTI_Dataset(dataset_path));
        dataset->readCalibData(); 

        //create map, local mapping nad visualizer objects and start their threads 
        local_mapping = std::shared_ptr<LocalMapping>(new LocalMapping);
        map = std::shared_ptr<Map>(new Map);
        visualizer = std::shared_ptr<Visualizer>(new Visualizer(false));

        // create bow vocabulary and start loop closer 
        if(use_loop_closing == true)
        {
            loop_closer = std::shared_ptr<LoopClosing>(new LoopClosing(vocab));
            vocab = std::shared_ptr<DBoW3::Vocabulary>(new DBoW3::Vocabulary(vocab_path)); 
        }

        //choose and set tracking algorithm 
        switch (algorithm_type)
        {
        case slamType::stereo_opf:
            left_camera = std::shared_ptr<Camera>(new Camera(dataset->P0, img_size_opt));
            right_camera = std::shared_ptr<Camera>(new Camera(dataset->P1, img_size_opt));

            stereoTracking = std::shared_ptr<StereoTracking_OPF>(new StereoTracking_OPF(TrackingType::ORB, use_loop_closing));
            stereoTracking->setTracking(map, local_mapping, loop_closer, visualizer, left_camera, right_camera, vocab);
            break;
        case slamType::stereo_matching: 
            left_camera = std::shared_ptr<Camera>(new Camera(dataset->P0, img_size_opt));
            right_camera = std::shared_ptr<Camera>(new Camera(dataset->P1, img_size_opt));

            stereoTracking_with_match = std::shared_ptr<StereoTracking_Match>(new StereoTracking_Match(TrackingType::GFTT));
            stereoTracking_with_match->setTracking(map, local_mapping, loop_closer, visualizer, left_camera, right_camera, vocab);
            break;  
        case slamType::mono: 
            //create camera objects (both camera are created even for mono slam because i don't want to change local mapping, right camera won't be used in practice)
            left_camera = std::shared_ptr<Camera>(new Camera(dataset->P2, img_size_opt)); // for monocular depth estimation you need color imgs 
            right_camera = std::shared_ptr<Camera>(new Camera(dataset->P3, img_size_opt));

            monoTracking = std::shared_ptr<MonoTracking>(new MonoTracking(TrackingType::GFTT, use_loop_closing)); 
            monoTracking->setTracking(map, local_mapping, loop_closer, visualizer, left_camera, vocab);
            break; 
        default:
            std::cout << "smth wrong \n"; 
            break;
        }

        // set local mapping and visualizer with pointers 
        local_mapping->setLocalMapping(map, left_camera, right_camera);
        visualizer->SetMap(map);
    }

    void SLAM::Run() 
    {
        std::cout << "Running main thread \n";

        //! main loop  
        while (true) 
        {
            if (createNewFrameAndTrack() == false) {
                break;
            }
        }

        if(loop_closer)
            loop_closer->end(); 

        local_mapping->Stop();
        visualizer->Close();
    }

    bool SLAM::createNewFrameAndTrack() 
    {
        /*
        read imgs, create Frame object and pass it to 
        */
        boost::format fmt("%s/image_%d/%06d.png");
        cv::Mat image_left, image_right;
        // read images

        auto beginT = std::chrono::steady_clock::now();

        image_left = cv::imread((fmt % dataset_path % 0 % current_image_index_).str(), cv::IMREAD_UNCHANGED);
        image_right = cv::imread((fmt % dataset_path % 1 % current_image_index_).str(), cv::IMREAD_UNCHANGED);

        if (image_left.data == nullptr || image_right.data == nullptr) 
        {
            std::cout << "cannot find images at index " << current_image_index_ << "\n";
            return false;
        }

        cv::Mat image_left_resized, image_right_resized;
        cv::resize(image_left, image_left_resized, cv::Size(), img_size_opt, img_size_opt, cv::INTER_NEAREST);
        cv::resize(image_right, image_right_resized, cv::Size(), img_size_opt, img_size_opt, cv::INTER_NEAREST);

        auto new_frame = Frame::CreateFrame();
        new_frame->left_img_ = image_left_resized;
        new_frame->right_img_ = image_right_resized;
        current_image_index_++;
        
        if (new_frame == nullptr) 
            return false;
        
        bool success = false; 

        switch (algorithm_type)
        {
        case slamType::stereo_opf:
            success = stereoTracking->AddFrame(new_frame);
            break;
        case slamType::stereo_matching: 
            success = stereoTracking_with_match->AddFrame(new_frame);
            break;  
        case slamType::mono: 
            success = monoTracking->AddFrame(new_frame);
            break; 
        }

        auto endT = std::chrono::steady_clock::now();
        auto elapsedT = std::chrono::duration_cast<std::chrono::milliseconds>(endT - beginT);
        std::cout  << "Loop time : " << elapsedT.count() << " ms. \n";

        performance.emplace_back(elapsedT.count());
        trajectory.emplace_back(new_frame->Pose().inverse().matrix3x4());

        return success;
    }

    void SLAM::output()
    {
        dataset->getGTposes(); 

        std::cout << "------- Results --------- \n"; 
        std::cout << "number of features used " << stereoTracking->num_features << "\n"; 
        std::cout << "number of created keyframes = " << map->getNumberOfKeyframes() << "\n"; 
        plotPerformance(performance);
        plotPoses(trajectory, dataset->ground_truth_poses, img_size_opt); 
        calculate_error(trajectory, dataset->ground_truth_poses, img_size_opt, 6); 
    } 

}  // namespace myslam
