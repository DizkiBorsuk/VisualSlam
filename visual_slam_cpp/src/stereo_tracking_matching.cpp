#include "myslam/stereo_tracking_matching.hpp"
#include "myslam/local_mapping.hpp"
#include "myslam/loop_closing.hpp"
#include "myslam/g2o_types.hpp"
#include "myslam/map.hpp"
#include "myslam/visualizer.hpp"

namespace myslam 
{
    StereoTracking_Match::StereoTracking_Match(TrackingType choose_tracking_type) 
    {
        type = choose_tracking_type; 
        switch(type)
        {
            case TrackingType::GFTT:
                detector = cv::GFTTDetector::create(num_features, 0.01, 20);
                extractor = cv::ORB::create(num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE);
                break; 
            case TrackingType::ORB: 
                detector = cv::ORB::create(num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20); // WTA_K can be change to 3 and 4 but then BRUTEFORCE_HAMMING myst be changed to BRUTEFORCE_HAMMINGLUT
                extractor = cv::ORB::create(num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE); 
                break;  
            case TrackingType::SIFT: 
                detector = cv::SIFT::create(num_features, 3, 0.04, 10, 1.6, false);  
                extractor = cv::SIFT::create(num_features, 3, 0.04, 10, 1.6, false); 
                break;
            default: 
                break; 
        }

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING); 
    }

    bool StereoTracking_Match::AddFrame(std::shared_ptr<Frame> frame) 
    {
        current_frame = frame;
        std::cout << "added frame \n"; 
        switch (status) {
            case TrackingStatus::INITING:
                StereoInit();
                break;
            case TrackingStatus::TRACKING:
                Track();
                break;
            case TrackingStatus::LOST:
                return false; 
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
        std::cout << "tracking points = " << num_track_last << "\n"; 
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
        
        extractStereoFeatures(); 
        TriangulateNewPoints();

        if(loop_closer)
            loop_closer->addCurrentKeyframe(current_frame); 

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
                    new_map_point->AddObservation(current_frame->features_left_[i]);
                    new_map_point->AddObservation(current_frame->features_right_[i]);

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

    int StereoTracking_Match::TrackLastFrame() 
    {
        cv::Mat mask(last_frame->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame->features_left_) 
        {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10), feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED); 
        }

        std::vector<cv::KeyPoint> kps_last, kps_current; 
        cv::Mat descriptors_current, descriptors_last;  // each row is diffrent descriptor 
        std::vector<cv::DMatch> matched_points, good_matches; 
        unsigned int found_features = 0; 
        
        // extract from current left img
        detector->detect(current_frame->left_img_, kps_current, mask);  
        extractor->compute(current_frame->left_img_, kps_current, descriptors_current); 
        
        //get descriptors from previous img 
        for(std::size_t i = 0; i < last_frame->features_left_.size(); i++)
        {
            descriptors_last.push_back(last_frame->features_left_[i]->descriptor); 
        }

        matcher->match(descriptors_current, descriptors_last, matched_points); 

        double min_dist=10000, max_dist=0;

        for ( int i = 0; i < descriptors_current.rows; i++ )
        {
            double dist = matched_points[i].distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        for(size_t i; i < matched_points.size(); i++ )
        {
            if ( matched_points[i].distance <= std::max( 2*min_dist, 30.0 ) )
            {
                good_matches.push_back ( matched_points[i] );
            }
        }

        for(unsigned int m = 0; m < good_matches.size(); m++)
        {
            std::shared_ptr<Feature> new_feature(new Feature(current_frame, kps_current[good_matches[m].queryIdx], descriptors_current.row(good_matches[m].queryIdx))); 
            auto mp = last_frame->features_left_[good_matches[m].trainIdx]->map_point_.lock(); 
            if(mp)
            {
                new_feature->map_point_ = last_frame->features_left_[good_matches[m].trainIdx]->map_point_; 
                current_frame->features_left_.emplace_back(new_feature); 
                found_features++;
            }
            else {continue;}
        }
        std::cout  << "found " << found_features << " in the last image. \n";
        return found_features; 
    }

    bool StereoTracking_Match::StereoInit() 
    {
        int num_coor_features = extractStereoFeatures();

        if (num_coor_features < num_features_init) {
            return false;
        }

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

    int StereoTracking_Match::extractStereoFeatures()
    {
        /*
            extract features from left and right img and match them with features
            then create both feature objects  
        */
        // initilize needed stuctures
        std::vector<cv::KeyPoint> kps_left, kps_right;
        cv::Mat descriptors_left, descriptors_right;  // each row is diffrent descriptor 
        std::vector<cv::DMatch> matched_points; 
        unsigned int found_features = 0; 

        // create mask to find correct features in left img
        cv::Mat mask(current_frame->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame->features_left_) 
        {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10), feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED); 
        }
        
        // extract from left img and create bow vector
        detector->detect(current_frame->left_img_, kps_left, mask);  
        extractor->compute(current_frame->left_img_, kps_left, descriptors_left); 

        if(vocabulary)
            vocabulary->transform(descriptors_left, current_frame->bow_vector); 

        //extract features from right img (no need for bow)
        detector->detect(current_frame->right_img_, kps_right, cv::noArray());  
        extractor->compute(current_frame->right_img_, kps_right, descriptors_right); 

        // match points and filter them based on distance 
        matcher->match(descriptors_left, descriptors_right, matched_points); 

        double min_dist=10000, max_dist=0;
        for (int i = 0; i < descriptors_left.rows; i++ )
        {
            double dist = matched_points[i].distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        for ( int i = 0; i < descriptors_left.rows; i++ )
        {
            if(matched_points[i].distance <= std::max( 2*min_dist, 30.0 ))
            {
                current_frame->features_left_.emplace_back(new Feature(current_frame, kps_left.at(matched_points[i].queryIdx), descriptors_left.row(matched_points[i].queryIdx))); 
                current_frame->features_right_.emplace_back(new Feature(current_frame, kps_right.at(matched_points[i].trainIdx), descriptors_right.row(matched_points[i].trainIdx), true)); 
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
            std::vector<Eigen::Vector3d> points{camera_left->pixel2camera(Eigen::Vector2d(current_frame->features_left_[i]->position_.pt.x, current_frame->features_left_[i]->position_.pt.y)),
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

    int StereoTracking_Match::EstimateCurrentPose() 
    {
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

        // estimate the Pose and determine the outliers
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

}  // namespace myslam