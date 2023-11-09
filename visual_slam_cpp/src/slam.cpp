
#include "myslam/slam.hpp"
#include <boost/config.hpp>
#include <boost/format.hpp>


namespace myslam 
{
    StereoSLAM::StereoSLAM(std::string &in_dataset_path, float resize)
        : dataset_path(in_dataset_path), img_size_opt(resize)
    {

    }

    void StereoSLAM::Init() 
    {
        dataset = std::shared_ptr<KITTI_Dataset>(new KITTI_Dataset(dataset_path));
        dataset->readCalibData(); 

        left_camera = std::shared_ptr<Camera>(new Camera(dataset->P0));
        right_camera = std::shared_ptr<Camera>(new Camera(dataset->P1));

        // create components and links
        stereoTracking = std::shared_ptr<StereoTracking>(new StereoTracking(TrackingType::Matching_ORB));
        local_mapping = std::shared_ptr<LocalMapping>(new LocalMapping);
        map = std::shared_ptr<Map>(new Map);
        visualizer = std::shared_ptr<Visualizer>(new Visualizer(false));

        stereoTracking->setTracking(map, local_mapping, visualizer, left_camera, right_camera);

        local_mapping->setLocalMapping(map, left_camera, right_camera);

        visualizer->SetMap(map);
    }

    void StereoSLAM::Run() 
    {
        std::cout << "Running main thread \n";

        while (true) 
        {
            if (createNewFrameAndTrack() == false) {
                break;
            }
        }

        local_mapping->Stop();
        visualizer->Close();
    }

    bool StereoSLAM::createNewFrameAndTrack() 
    {
        boost::format fmt("%s/image_%d/%06d.png");
        cv::Mat image_left, image_right;
        // read images

        auto beginT = std::chrono::steady_clock::now();

        image_left = cv::imread((fmt % dataset_path % 0 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);
        image_right = cv::imread((fmt % dataset_path % 1 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);

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
    
        bool success = stereoTracking->AddFrame(new_frame);
        auto endT = std::chrono::steady_clock::now();
        auto elapsedT = std::chrono::duration_cast<std::chrono::milliseconds>(endT - beginT);
        std::cout  << "Loop time : " << elapsedT.count() << " ms. \n";

        performance.emplace_back(elapsedT.count());
        trajectory.emplace_back(new_frame->Pose().inverse().matrix3x4());

        return success;
    }

    void StereoSLAM::output()
    {
        dataset->getGTposes(); 

        std::cout << "------- Results --------- \n"; 

        plotPerformance(performance);
        plotPoses(trajectory, dataset->ground_truth_poses); 
        calculate_error(trajectory, dataset->ground_truth_poses); 
        std::cout << "number of features used " << stereoTracking->num_features << "\n"; 
        
    } 

}  // namespace myslam
