#include "myslam/stereo_tracking_matching.hpp"
#include "myslam/local_mapping.hpp"
#include "myslam/loop_closing.hpp"
#include "myslam/g2o_types.hpp"
#include "myslam/map.hpp"
#include "myslam/visualizer.hpp"

namespace myslam 
{
    StereoTracking_Match::StereoTracking_Match(TrackingType choose_tracking_type, bool descriptors) 
    {
        type = choose_tracking_type; 
        use_descriptors = descriptors; 
        switch(type)
        {
            case TrackingType::GFTT:
                detector = cv::GFTTDetector::create(num_features, 0.01, 20);
                if(use_descriptors == true) 
                    extractor = cv::ORB::create(num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE);
                break; 
            case TrackingType::ORB: 
                detector = cv::ORB::create(num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20); // WTA_K can be change to 3 and 4 but then BRUTEFORCE_HAMMING myst be changed to BRUTEFORCE_HAMMINGLUT
                if(use_descriptors == true) 
                    extractor = cv::ORB::create(num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE); 
                break; 
            case TrackingType::FAST_ORB: 
                detector = cv::FastFeatureDetector::create(10, true, cv::FastFeatureDetector::TYPE_9_16);  
                if(use_descriptors == true) 
                    extractor = cv::ORB::create(num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE); 
                break; 
            case TrackingType::SIFT: 
                detector = cv::SIFT::create(num_features, 3, 0.04, 10, 1.6, false);  
                if(use_descriptors == true) 
                    extractor = cv::SIFT::create(num_features, 3, 0.04, 10, 1.6, false); 
                break;
            default: 
                break; 
        }
    }

    bool StereoTracking_Match::AddFrame(std::shared_ptr<Frame> frame) 
    {
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

    bool StereoTracking_Match::Track() 
    {
        if (last_frame) 
        {
            current_frame->SetPose(relative_motion_ * last_frame->Pose());
        }

        int num_track_last = TrackLastFrame();
        std::cout << "bunver if tracking points = " << num_track_last << "\n"; 
        tracking_inliers_ = EstimateCurrentPose();

        if (tracking_inliers_ > num_features_tracking_bad_) {
            // tracking good
            status = TrackingStatus::TRACKING;
        } else {
            // lost
            status = TrackingStatus::LOST;
            return false; 
        }

        if (tracking_inliers_ < num_features_needed_for_keyframe) {
            InsertKeyframe();
        }

        relative_motion_ = current_frame->Pose() * (last_frame->Pose().inverse());

        visualizer->AddCurrentFrame(current_frame);
        return true;
    }

    bool StereoTracking_Match::InsertKeyframe() {

        // current frame is a new keyframe
        current_frame->SetKeyFrame();
        map->InsertKeyFrame(current_frame);

        std::cout  << "Set frame " << current_frame->id << " as keyframe " << current_frame->keyframe_id << "\n";

        for (auto &feat : current_frame->features_left_) 
        {
            auto mp = feat->map_point_.lock();
            if (mp) mp->AddObservation(feat);
        }
        
        extractFeatures(); 
        loop_closer->addCurrentKeyframe(current_frame); 

        // track in right image

        // triangulate map points
        TriangulateNewPoints();
        // update backend because we have a new keyframe
        local_mapping->UpdateMap();
        visualizer->UpdateMap();

        return true;
    }

    int StereoTracking_Match::TriangulateNewPoints() {

        Sophus::SE3d current_pose_Twc = current_frame->Pose().inverse();
        int cnt_triangulated_pts = 0;
        for (std::size_t i = 0; i < current_frame->features_left_.size(); i++) 
        {
            if (current_frame->features_left_[i]->map_point_.expired() &&
                current_frame->features_right_[i] != nullptr) {
                std::vector<Eigen::Vector3d> points{camera_left->pixel2camera(Eigen::Vector2d(current_frame->features_left_[i]->position_.pt.x,current_frame->features_left_[i]->position_.pt.y)),
                                                    camera_right->pixel2camera(Eigen::Vector2d(current_frame->features_right_[i]->position_.pt.x, current_frame->features_right_[i]->position_.pt.y))};
                Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

                if (triangulation(camera_left->pose(),camera_right->pose(), points, pworld)) 
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

    int StereoTracking_Match::EstimateCurrentPose() {
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
        Eigen::Matrix3d K = camera_left->getK();

        // edges
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<std::shared_ptr<Feature>> features;

        // edges.reserve(current_frame->features_left_.size()); 
        // features.reserve(current_frame->features_left_.size()); 

        for (size_t i = 0; i < current_frame->features_left_.size(); ++i) 
        {
            auto mp = current_frame->features_left_[i]->map_point_.lock();
            if (mp) 
            {
                features.emplace_back(current_frame->features_left_[i]);
                EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(toVec2(current_frame->features_left_[i]->position_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.emplace_back(edge);
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

        std::cout  << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/" << features.size() - cnt_outlier << "\n";
        // Set pose and outlier
        current_frame->SetPose(vertex_pose->estimate());

        //std::cout  << "Current Pose = \n" << current_frame->Pose().matrix() << "\n";

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

    int StereoTracking_Match::TrackLastFrame() {
        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_last, kps_current;

        for (auto &kp : last_frame->features_left_) {
            if (kp->map_point_.lock()) 
            {
                // use projected point
                auto mp = kp->map_point_.lock();
                auto px = camera_left->world2pixel(mp->pos_, current_frame->Pose());
                kps_last.emplace_back(kp->position_.pt);
                kps_current.emplace_back(cv::Point2f(px[0], px[1]));
            } else {
                kps_last.emplace_back(kp->position_.pt);
                kps_current.emplace_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK( last_frame->left_img_, current_frame->left_img_, kps_last,
                                  kps_current, status, error, cv::Size(11, 11), 3,
                                  cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;

        for (size_t i = 0; i < status.size(); ++i) 
        {
            if (status[i]) {
                cv::KeyPoint kp(kps_current[i], 7);
                std::shared_ptr<Feature> feature(new Feature(current_frame, kp));
                feature->map_point_ = last_frame->features_left_[i]->map_point_;
                current_frame->features_left_.emplace_back(feature);
                num_good_pts++;
            }
        }

        std::cout  << "Find " << num_good_pts << " in the last image. \n";
        return num_good_pts;
    }

    bool StereoTracking_Match::StereoInit() 
    {
   
        extractFeatures(); 
 
        // int num_coor_features;

        // if (num_coor_features < num_features_init) {
        //     return false;
        // }

        if (BuildInitMap()) {
            
            status = TrackingStatus::TRACKING;

            current_frame->SetKeyFrame();
            map->InsertKeyFrame(current_frame);
            local_mapping->UpdateMap();

            visualizer->AddCurrentFrame(current_frame);
            visualizer->UpdateMap();
            if(loop_closer)
                loop_closer->addCurrentKeyframe(current_frame); 
        
            return true;
        }
        return false;
    }

    int StereoTracking_Match::extractFeatures()
    {
        /*
        Extract features (keypoint and descriptor) from left img 
        */
        cv::Mat mask(current_frame->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame->features_left_) 
        {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10), feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED); 
        }

        unsigned int detected_features = 0;

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;  // each row is diffrent descriptor 
        
        // extract from left img and create bow 
        detector->detect(current_frame->left_img_, keypoints, mask);  
        extractor->compute(current_frame->left_img_, keypoints, descriptors); 
        vocabulary->transform(descriptors, current_frame->bow_vector); 
        
        for(size_t i = 0; i < keypoints.size(); i++)
        {
            current_frame->features_left_.emplace_back(new Feature(current_frame, keypoints.at(i), descriptors.row(i))); 
            detected_features++;
        }

        return detected_features;
    }

    int StereoTracking_Match::extractStereoFeatures()
    {
        /*
            extract features from left andright img and match them with features then create both feature objects 
        */
        std::vector<cv::KeyPoint> kps_left, kps_right;
        cv::Mat descriptors_left, descriptors_right; 
        std::vector<std::vector<cv::DMatch>> matched_points; 

        // extract from left img and create bow 
        detector->detect(current_frame->left_img_, kps_left, cv::noArray());  
        extractor->compute(current_frame->left_img_, kps_left, descriptors_left); 
        vocabulary->transform(descriptors_left, current_frame->bow_vector); 

        //extract features from right img (no need for bow)
        detector->detect(current_frame->right_img_, kps_right, cv::noArray());  
        extractor->compute(current_frame->right_img_, kps_right, descriptors_right); 

        matcher->knnMatch(descriptors_right, descriptors_right, matched_points, 2); 

        float low_ratio = 0.7f; 
        std::vector<std::vector<cv::DMatch>>::iterator it;

        unsigned int found_features = 0; 
        for (it= matched_points.begin(); it!= matched_points.end(); ++it) 
        {
            if ((*it)[0].distance/(*it)[1].distance < low_ratio)
            {
                current_frame->features_left_.emplace_back(new Feature(current_frame, kps_left.at((*it)[0].queryIdx), descriptors_left.row((*it)[0].queryIdx))); 
                current_frame->features_right_.emplace_back(new Feature(current_frame, kps_left.at((*it)[0].queryIdx), descriptors_left.row((*it)[0].queryIdx), true)); 
                found_features++;
            }
        }
        return found_features; 
    }

    bool StereoTracking_Match::BuildInitMap() 
    {
        unsigned int cnt_init_landmarks = 0;
        for (size_t i = 0; i < current_frame->features_left_.size(); ++i) 
        {
            if (current_frame->features_right_[i] == nullptr)
            {
                continue;
            }

            // create map point from triangulation
            std::vector<Eigen::Vector3d> points{ camera_left->pixel2camera(Eigen::Vector2d(current_frame->features_left_[i]->position_.pt.x, current_frame->features_left_[i]->position_.pt.y)),
                                                camera_right->pixel2camera(Eigen::Vector2d(current_frame->features_right_[i]->position_.pt.x,current_frame->features_right_[i]->position_.pt.y))};
            Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

            if (triangulation(camera_left->pose(),camera_right->pose(), points, pworld)) {
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
        std::cout  << "Initial map created with " << cnt_init_landmarks
                << " map points \n";

        return true;
    }

    bool StereoTracking_Match::Reset() {
        std::cout  << "Tracking lost \n";
        return false;
    }


}  // namespace myslam