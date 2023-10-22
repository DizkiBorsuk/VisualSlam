#include "myslam/local_mapping.h"
#include "myslam/stereo_tracking.hpp"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

namespace myslam {

    StereoTracking::StereoTracking(TrackingType choose_tracking_type) 
    {
        type = choose_tracking_type; 
        switch(type)
        {
            case TrackingType::OpticalFlow_GFTT:
                detector = cv::GFTTDetector::create(num_features, 0.01, 20);
                break; 
            case TrackingType::OpticalFlow_ORB: 
                detector = cv::ORB::create(num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::FAST_SCORE);
                break; 
            case TrackingType::Matching: 
                matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

                detector = cv::ORB::create(num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::FAST_SCORE);  
                extractor = cv::ORB::create(num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::FAST_SCORE); 
                break; 
            default: 
                break; 
        }
    }

    bool StereoTracking::AddFrame(std::shared_ptr<Frame> frame) {
        current_frame = frame;

        switch (status) {
            case TrackingStatus::INITING:
                StereoInit();
                break;
            case TrackingStatus::TRACKING:
                Track();
                break;
            case TrackingStatus::LOST:
                Reset();
                break;
        }

        last_frame = current_frame;
        return true;
    }

    bool StereoTracking::Track() {
        if (last_frame) {
            current_frame->SetPose(relative_motion_ * last_frame->Pose());
        }

        int num_track_last = TrackLastFrame();
        tracking_inliers_ = EstimateCurrentPose();

        if (tracking_inliers_ > num_features_tracking_bad_) {
            // tracking good
            status = TrackingStatus::TRACKING;
        } else {
            // lost
            status = TrackingStatus::LOST;
        }

        if (tracking_inliers_ < num_features_needed_for_keyframe_) {
            InsertKeyframe();
        }

        relative_motion_ = current_frame->Pose() * last_frame->Pose().inverse();

        if (viewer_) viewer_->AddCurrentFrame(current_frame);
        return true;
    }

    bool StereoTracking::InsertKeyframe() {

        // current frame is a new keyframe
        current_frame->SetKeyFrame();
        map->InsertKeyFrame(current_frame);

        std::cout  << "Set frame " << current_frame->id_ << " as keyframe "
                << current_frame->keyframe_id_ << "\n";

        for (auto &feat : current_frame->features_left_) 
        {
            auto mp = feat->map_point_.lock();
            if (mp) mp->AddObservation(feat);
        }
        
        DetectFeatures();  // detect new features

        // track in right image
        FindFeaturesInRight();
        // triangulate map points
        TriangulateNewPoints();
        // update backend because we have a new keyframe
        local_mapping->UpdateMap();

        viewer_->UpdateMap();

        return true;
    }

    int StereoTracking::TriangulateNewPoints() {

        Sophus::SE3d current_pose_Twc = current_frame->Pose().inverse();
        int cnt_triangulated_pts = 0;
        for (std::size_t i = 0; i < current_frame->features_left_.size(); i++) 
        {
            if (current_frame->features_left_[i]->map_point_.expired() &&
                current_frame->features_right_[i] != nullptr) {
                std::vector<Eigen::Vector3d> points{
                    camera_left_->pixel2camera(
                        Eigen::Vector2d(current_frame->features_left_[i]->position_.pt.x,
                            current_frame->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(
                        Eigen::Vector2d(current_frame->features_right_[i]->position_.pt.x,
                            current_frame->features_right_[i]->position_.pt.y))};
                Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

                if (triangulation(camera_left_->pose(),camera_right_->pose(), points, pworld)) 
                {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    pworld = current_pose_Twc * pworld;
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(
                        current_frame->features_left_[i]);
                    new_map_point->AddObservation(
                        current_frame->features_right_[i]);

                    current_frame->features_left_[i]->map_point_ = new_map_point;
                    current_frame->features_right_[i]->map_point_ = new_map_point;
                    map->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        std::cout  << "new landmarks: " << cnt_triangulated_pts << "\n";
        return cnt_triangulated_pts;
    }

    int StereoTracking::EstimateCurrentPose() {
        // setup g2o
        typedef g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolver_6_3>(std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // vertex
        VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame->Pose());
        optimizer.addVertex(vertex_pose);

        // K
        Eigen::Matrix3d K = camera_left_->K();

        // edges
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<std::shared_ptr<Feature>> features;

        for (size_t i = 0; i < current_frame->features_left_.size(); ++i) {
            auto mp = current_frame->features_left_[i]->map_point_.lock();
            if (mp) {
                features.push_back(current_frame->features_left_[i]);
                EdgeProjectionPoseOnly *edge =
                    new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(toVec2(current_frame->features_left_[i]->position_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }

        // estimate the Pose the determine the outliers
        const double chi2_th = 5.991;
        int cnt_outlier = 0;
        for (int iteration = 0; iteration < 4; ++iteration) 
        {
            vertex_pose->setEstimate(current_frame->Pose());
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;

            // count the outliers
            for (size_t i = 0; i < edges.size(); ++i) 
            {
                auto e = edges[i];
                if (features[i]->is_outlier_) 
                {
                    e->computeError();
                }
                if (e->chi2() > chi2_th) 
                {
                    features[i]->is_outlier_ = true;
                    e->setLevel(1);
                    cnt_outlier++;
                } else {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);
                };

                if (iteration == 2) 
                {
                    e->setRobustKernel(nullptr);
                }
            }
        }

        std::cout  << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
                << features.size() - cnt_outlier << "\n";
        // Set pose and outlier
        current_frame->SetPose(vertex_pose->estimate());

        std::cout  << "Current Pose = \n" << current_frame->Pose().matrix() << "\n";

        for (auto &feat : features) 
        {
            if (feat->is_outlier_) 
            {
                feat->map_point_.reset();
                feat->is_outlier_ = false;  // maybe we can still use it in future
            }
        }
        return features.size() - cnt_outlier;
    }

    int StereoTracking::TrackLastFrame() {
        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_last, kps_current;
        for (auto &kp : last_frame->features_left_) {
            if (kp->map_point_.lock()) {
                // use project point
                auto mp = kp->map_point_.lock();
                auto px = camera_left_->world2pixel(mp->pos_, current_frame->Pose());
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(px[0], px[1]));
            } else {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(
            last_frame->left_img_, current_frame->left_img_, kps_last,
            kps_current, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                            0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;

        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                cv::KeyPoint kp(kps_current[i], 7);
                std::shared_ptr<Feature> feature(new Feature(current_frame, kp));
                feature->map_point_ = last_frame->features_left_[i]->map_point_;
                current_frame->features_left_.push_back(feature);
                num_good_pts++;
            }
        }

        std::cout  << "Find " << num_good_pts << " in the last image. \n";
        return num_good_pts;
    }

    bool StereoTracking::StereoInit() {
        DetectFeatures();
        int num_coor_features = FindFeaturesInRight();
        if (num_coor_features < num_features_init) {
            return false;
        }

        if (BuildInitMap()) {
            
            status = TrackingStatus::TRACKING;
            viewer_->AddCurrentFrame(current_frame);
            viewer_->UpdateMap();
        
            return true;
        }
        return false;
    }

    int StereoTracking::DetectFeatures() 
    {
        cv::Mat mask(current_frame->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame->features_left_) {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                        feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
        }

        std::vector<cv::KeyPoint> keypoints;
        detector->detect(current_frame->left_img_, keypoints, mask);
        unsigned int detected_features = 0;

        for (auto &kp : keypoints) 
        {
            current_frame->features_left_.emplace_back(new Feature(current_frame, kp));
            detected_features++;
        }

        std::cout  << "Detect " << detected_features << " new features \n";
        return detected_features;
    }

    int StereoTracking::ExtractFeatures()
    {
        unsigned int detected_features = 0;

        std::vector<cv::KeyPoint> keypoints; 
        cv::Mat descriptors; // each row is diffrent descriptor 

        detector->detect(current_frame->left_img_, keypoints, cv::noArray());  
        extractor->compute(current_frame->left_img_, keypoints, descriptors); 

        for(size_t i = 0; i < keypoints.size(); i++)
        {
            current_frame->features_left_.emplace_back(new Feature(current_frame, keypoints.at(i), descriptors.row(i))); 
        }

        return detected_features;
    }


    int StereoTracking::FindFeaturesInRight() {
        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_left, kps_right;
        for (auto &kp : current_frame->features_left_) {
            kps_left.push_back(kp->position_.pt);
            auto mp = kp->map_point_.lock();
            if (mp) {
                // use projected points as initial guess
                auto px =
                    camera_right_->world2pixel(mp->pos_, current_frame->Pose());
                kps_right.push_back(cv::Point2f(px[0], px[1]));
            } else {
                // use same pixel in left iamge
                kps_right.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(
            current_frame->left_img_, current_frame->right_img_, kps_left,
            kps_right, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                            0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                cv::KeyPoint kp(kps_right[i], 7);
                std::shared_ptr<Feature> feat(new Feature(current_frame, kp));
                feat->is_on_left_image_ = false;
                current_frame->features_right_.push_back(feat);
                num_good_pts++;
            } else {
                current_frame->features_right_.push_back(nullptr);
            }
        }
        std::cout  << "Find " << num_good_pts << " in the right image. \n";
        return num_good_pts;
    }

    bool StereoTracking::BuildInitMap() 
    {
        unsigned int cnt_init_landmarks = 0;
        for (size_t i = 0; i < current_frame->features_left_.size(); ++i) 
        {
            if (current_frame->features_right_[i] == nullptr)
            {
                continue;
            }

            // create map point from triangulation
            std::vector<Eigen::Vector3d> points{
                camera_left_->pixel2camera(
                    Eigen::Vector2d(current_frame->features_left_[i]->position_.pt.x,
                        current_frame->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Eigen::Vector2d(current_frame->features_right_[i]->position_.pt.x,
                        current_frame->features_right_[i]->position_.pt.y))};
            Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

            if (triangulation(camera_left_->pose(),camera_right_->pose(), points, pworld)) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(current_frame->features_left_[i]);
                new_map_point->AddObservation(current_frame->features_right_[i]);
                current_frame->features_left_[i]->map_point_ = new_map_point;
                current_frame->features_right_[i]->map_point_ = new_map_point;
                cnt_init_landmarks++;
                map->InsertMapPoint(new_map_point);
            }
        }
        current_frame->SetKeyFrame();
        map->InsertKeyFrame(current_frame);
        local_mapping->UpdateMap();

        std::cout  << "Initial map created with " << cnt_init_landmarks
                << " map points \n";

        return true;
    }

    bool StereoTracking::Reset() {
        std::cout  << "Tracking lost \n";
        return false;
    }

}  // namespace myslam