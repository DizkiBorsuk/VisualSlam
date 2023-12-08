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

                detector_init = cv::GFTTDetector::create(1000, 0.01, 20);
                extractor_init = cv::ORB::create(1000, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE);
                break; 
            case TrackingType::ORB: 
                detector = cv::ORB::create(num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20); // WTA_K can be change to 3 and 4 but then BRUTEFORCE_HAMMING myst be changed to BRUTEFORCE_HAMMINGLUT
                extractor = cv::ORB::create(num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE); 

                detector_init = cv::ORB::create(1000, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20); // WTA_K can be change to 3 and 4 but then BRUTEFORCE_HAMMING myst be changed to BRUTEFORCE_HAMMINGLUT
                extractor_init = cv::ORB::create(1000, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE);
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
        auto beginT = std::chrono::steady_clock::now();
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
        auto endT = std::chrono::steady_clock::now();
        auto elapsedT = std::chrono::duration_cast<std::chrono::milliseconds>(endT - beginT);
        std::cout  << "Loop time for tracking: " << elapsedT.count() << " ms. \n";
        return true;
    }

    void StereoTracking_Match::StereoInit() 
    {
        int num_coor_features = extractStereoFeatures();
    
        if (num_coor_features < num_features_init) {
            return;
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
        
        }
    }

    bool StereoTracking_Match::BuildInitMap() 
    {
        unsigned int triangulated_mappoints = 0;
        for (size_t i = 0; i < current_frame->features_left_.size(); ++i) 
        {
            // create map point from triangulation
            Eigen::Vector3d left_point = camera_left->pixel2camera(Eigen::Vector2d(current_frame->features_stereo[i]->position_in_left.pt.x, current_frame->features_stereo[i]->position_in_left.pt.y)); 
            Eigen::Vector3d right_point = camera_right->pixel2camera(Eigen::Vector2d(current_frame->features_stereo[i]->position_in_right.pt.x,current_frame->features_stereo[i]->position_in_right.pt.y));

            std::vector<Eigen::Vector3d> points{left_point, right_point};

            Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

            if (triangulation(camera_left->pose(),camera_right->pose(), points, pworld)) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(current_frame->features_stereo[i]);
            
                current_frame->features_stereo[i]->map_point_ = new_map_point;
                triangulated_mappoints++;
                map->InsertMapPoint(new_map_point);
            }
        }
        std::cout  << "Initial map created with " << triangulated_mappoints << " map points \n";
        return true;
    }

    bool StereoTracking_Match::Track() 
    {
        if (last_frame) 
        {
            current_frame->SetPose(relative_motion_ * last_frame->getPose());
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

        relative_motion_ = current_frame->getPose() * (last_frame->getPose().inverse());

        visualizer->AddCurrentFrame(current_frame);
        return true;
    }

    bool StereoTracking_Match::InsertKeyframe() {

        // current frame is a new keyframe
        current_frame->SetKeyFrame();
        map->InsertKeyFrame(current_frame);

        std::cout  << "Set frame " << current_frame->id << " as keyframe " << current_frame->keyframe_id << "\n";

        for (auto &feat : current_frame->features_stereo) 
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

        Sophus::SE3d current_pose_Twc = current_frame->getPose().inverse();
        int triangulated_pts = 0;
        
        std::cout << "left features size = " << current_frame->features_left_.size() << "\n"; 
        std::cout << "right features size = " << current_frame->features_right_.size() << "\n"; 

        for (std::size_t i = 0; i < current_frame->features_stereo.size(); i++) 
        {
            if (current_frame->features_stereo.at(i)->map_point_.expired()) 
            {
                Eigen::Vector3d left_point = camera_left->pixel2camera(Eigen::Vector2d(current_frame->features_stereo[i]->position_in_left.pt.x, current_frame->features_stereo[i]->position_in_left.pt.y)); 
                Eigen::Vector3d right_point = camera_right->pixel2camera(Eigen::Vector2d(current_frame->features_stereo[i]->position_in_right.pt.x,current_frame->features_stereo[i]->position_in_right.pt.y));

                std::vector<Eigen::Vector3d> points{left_point, right_point};
                
                Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

                if (triangulation(camera_left->pose(),camera_right->pose(), points, pworld)) 
                {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    pworld = current_pose_Twc * pworld;
                    
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(current_frame->features_stereo[i]);
                    current_frame->features_stereo[i]->map_point_ = new_map_point;
                    map->InsertMapPoint(new_map_point);
                    triangulated_pts++;
                }
            }
        }
        std::cout  << "new landmarks: " << triangulated_pts << "\n";
        return triangulated_pts;
    }

    int StereoTracking_Match::TrackLastFrame() 
    {
        /*
            track features from previos frame to current frame 
        */

        cv::Mat descriptors_current_left, descriptors_current_right, descriptors_last_left;  // each row is diffrent descriptor 
        std::vector<cv::DMatch> matches_current_lr, good_matches_current_lr, matches_prev_current, good_matches; 
        unsigned int found_features = 0; 
        
        //get descriptors from previous frame 
        for(std::size_t i = 0; i < last_frame->features_stereo.size(); i++)
        {
            descriptors_last_left.push_back(last_frame->features_stereo[i]->descriptor_left); 
        }
    
        //! extract features from current img 
        std::vector<cv::KeyPoint> kps_left, kps_right;

        //create mask to find correct features in left img
        cv::Mat mask_left(last_frame->left_img_.size(), CV_8UC1, 255);
        cv::Mat mask_right(last_frame->right_img_.size(), CV_8UC1, 255);
        for (auto &feat : last_frame->features_stereo) 
        {
            cv::rectangle(mask_left, feat->position_in_left.pt - cv::Point2f(10, 10), feat->position_in_left.pt + cv::Point2f(10, 10), 0, cv::FILLED); 
            cv::rectangle(mask_right, feat->position_in_right.pt - cv::Point2f(10, 10), feat->position_in_right.pt + cv::Point2f(10, 10), 0, cv::FILLED); 
        }
        
        // extract from left img and create bow vector
        detector->detect(current_frame->left_img_, kps_left, mask_left);  
        extractor->compute(current_frame->left_img_, kps_left, descriptors_current_left); 

        //extract features from right img (no need for bow)
        detector->detect(current_frame->right_img_, kps_right, mask_right);  
        extractor->compute(current_frame->right_img_, kps_right, descriptors_current_right); 

        //! match features current img
        matcher->match(descriptors_current_left, descriptors_current_right, matches_current_lr); 

        double min_dist=10000, max_dist=0;
        for ( std::size_t i = 0; i < matches_current_lr.size(); i++ )
        {
            double dist = matches_current_lr[i].distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        cv::Mat good_descriptors_current; 
        for(size_t i = 0; i < matches_current_lr.size(); i++ )
        {
            if ( matches_current_lr[i].distance <= std::max( 2*min_dist, 20.0 ) )
            {
                good_matches_current_lr.push_back(matches_current_lr[i]);
                good_descriptors_current.push_back(descriptors_current_left.row(matches_current_lr[i].queryIdx)); 
            }
        }

        // //! match good matches from current frame (left) with good matches from prev frame (left)
        matcher->match(descriptors_last_left, good_descriptors_current, matches_prev_current); 

        for(unsigned int m = 0; m < good_matches_current_lr.size(); m++)
        {
            
            good_descriptors_current.push_back(descriptors_current_left.row(matches_current_lr[m].queryIdx));

            // std::shared_ptr<Feature> new_feature(new Feature(current_frame, kps_current[good_matches[m].queryIdx], descriptors_current.row(good_matches[m].queryIdx))); 
            // auto mp = last_frame->features_left_[good_matches[m].trainIdx]->map_point_.lock(); 
            // if(mp)
            // {
            //     new_feature->map_point_ = last_frame->features_left_[good_matches[m].trainIdx]->map_point_; 
            //     current_frame->features_left_.emplace_back(new_feature); 
            //     found_features++;
            // }
            // else {continue;}
        }





        std::cout  << "found " << found_features << " in the last image. \n";
        return found_features; 
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

        //create mask to find correct features in left img
        cv::Mat mask_left(current_frame->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame->features_left_) 
        {
            cv::rectangle(mask_left, feat->position_.pt - cv::Point2f(10, 10), feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED); 
        }
        //create mask to find correct features in right img
        cv::Mat mask_right(current_frame->right_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame->features_right_) 
        {
            cv::rectangle(mask_right, feat->position_.pt - cv::Point2f(10, 10), feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED); 
        }
        
        // extract from left img and create bow vector
        detector->detect(current_frame->left_img_, kps_left, mask_left);  
        extractor->compute(current_frame->left_img_, kps_left, descriptors_left); 

        if(vocabulary)
            vocabulary->transform(descriptors_left, current_frame->bow_vector); 

        //extract features from right img (no need for bow)
        detector->detect(current_frame->right_img_, kps_right, mask_right);  
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

        for (std::size_t i = 0; i < matched_points.size(); i++ )
        {
            if(matched_points[i].distance <= std::max( 2*min_dist, 20.0 ))
            {
                found_features++; 
                current_frame->features_stereo.emplace_back(new FeatureStereo(current_frame, kps_left.at(matched_points[i].queryIdx), 
                                                                                             kps_right.at(matched_points[i].trainIdx), 
                                                                                             descriptors_left.row(matched_points[i].queryIdx), 
                                                                                             descriptors_right.row(matched_points[i].trainIdx))); 
            }
        }
        return found_features; 
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
        vertex_pose->setEstimate(current_frame->getPose());
        optimizer.addVertex(vertex_pose);

        // K
        Eigen::Matrix3d K = camera_left->getK();

        // edges
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<std::shared_ptr<FeatureStereo>> features;

        // edges.reserve(current_frame->features_left_.size()); 
        // features.reserve(current_frame->features_left_.size()); 

        for (size_t i = 0; i < current_frame->features_stereo.size(); ++i) 
        {
            auto mp = current_frame->features_stereo[i]->map_point_.lock();
            if (mp) 
            {
                features.emplace_back(current_frame->features_stereo[i]);
                EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(toVec2(current_frame->features_stereo[i]->position_in_left.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.emplace_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }

        // estimate the Pose and determine the outliers
        //const float chi2_th = 5.991; 
        const float chi2_th[4]={7.815,7.815,7.815, 7.815};

        int cnt_outlier = 0;
        for (int iteration = 0; iteration < 4; ++iteration) 
        {
            vertex_pose->setEstimate(current_frame->getPose());
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
                if (e->chi2() > chi2_th[iteration]) 
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