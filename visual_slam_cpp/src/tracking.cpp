#include "../include/tracking.hpp"
#include "../include/visualizer.hpp"
#include "../include/local_mapping.hpp"
#include "../include/tools.hpp"
#include "../include/graph_structure.hpp"

using namespace std::chrono_literals;

namespace mrVSLAM
{

    //---- Constructors and main functions ----- //
    Tracking::Tracking(DETECTOR detector_type)
    {
        switch (detector_type)
        {
        case DETECTOR::GFTT:
            detector = cv::GFTTDetector::create(num_of_features, 0.01, 20, 3, false, 0.04); //? big min distance
            break;
        case DETECTOR::FAST: 
            detector = cv::FastFeatureDetector::create(40, true, cv::FastFeatureDetector::TYPE_9_16); 
            break; 
        default:
            break;
        }
    }

    Tracking::Tracking(EXTRACTOR extractor_type)
    {
        switch (extractor_type)
        {
        case EXTRACTOR::ORB:
            detector = cv::GFTTDetector::create(num_of_features, 0.01, 20, 3, false, 0.04); //? big min distance
            break;
        case EXTRACTOR::SIFT: 
            detector = cv::FastFeatureDetector::create(40, true, cv::FastFeatureDetector::TYPE_9_16); 
            break; 
        case EXTRACTOR::AKAZE: 
            break; 
        default:
            break;
        }
    }

    void Tracking::setTracking(std::shared_ptr<Map> in_map, std::shared_ptr<Visualizer> in_visualizer, std::shared_ptr<LocalMapping> in_lMapping, 
                        std::shared_ptr<Camera> in_camera_left, std::shared_ptr<Camera> in_camera_right)
    {
        // set pointers to map, visualizer and backend 
        map = in_map; 
        visualizer = in_visualizer; 
        l_mapping = in_lMapping; 
        camera_right = in_camera_right; 
        camera_left = in_camera_left; 
    }

    bool Tracking::addFrameAndTrackStereo(std::shared_ptr<Frame> frame_to_add)
    {
        // get frame, set logic 
        current_frame = frame_to_add; 

        switch(tracking_status)
        {
            case STATUS::INITIALIZATION: 
                
                initSucces = stereoInitialize(); 

                if(initSucces== true) 
                {  
                    tracking_status = STATUS::TRACKING; 
                    std::cout << "status changed to Tracking\n";
                }
                break; 
            case STATUS::TRACKING: 
                track(); 
                break; 
            case STATUS::LOST: 
                restartTracking(); 
                break; 
        }
         
        prev_frame = current_frame; 
        // std::this_thread::sleep_for(30ms); 
        return true; 
    }

//########################################################
//---- Initialization ----- //

    bool Tracking::initialize()
    {
        // can't initialize stereo without at least 2 frames //? how to get two frames
        
        return false; 
    }

    bool Tracking::stereoInitialize()
    {
        std::cout << "started initialization \n";  
        //!DONE
        // initialize stereo tracking 
        unsigned int num_of_features_in_left_img = detectFeatures(); 
        unsigned int num_of_corresponding_features_in_right = findCorrespondingStereoFeatures(); 
        std::cout << "found " << num_of_corresponding_features_in_right <<" corresponding points \n"; 

        if(num_of_corresponding_features_in_right < num_of_features_for_initialization)
        {
            std::cout << "not succesful initialization"; 
            return false; //initialization failed 
        }
            
        
        buildMap(); 
        if(visualizer != nullptr || l_mapping !=nullptr)
        {
            l_mapping->updateMap(); 
            visualizer->addNewFrame(current_frame); 
            visualizer->getMapUpdate();
            std::cout << "succesful initialization \n";
            std::cout << "-------------------------- \n"; 
            return true; // initiaization succeded 
        }
        std::cout << "not succesful initialization"; 
        return false; //initialization failed  
    }

//########################################################
//----  ----- //

    unsigned int Tracking::detectFeatures()
    {
        //!DONE
        // function detects keypoints in main(left) img and pushes Features to Frame object 

        std::vector<cv::KeyPoint> keypoints; 
        detector->detect(current_frame->imgLeft, keypoints, cv::noArray()); 
        unsigned int detected_features = 0; 

        for(auto &point : keypoints)
        {
            current_frame->featuresFromLeftImg.emplace_back(new Feature{current_frame, point}); 
            detected_features++; 
        }

        return detected_features; 
    }

    unsigned int Tracking::extractFeatures()
    {
        //? have to check if correct
        /*
        */
        std::vector<cv::KeyPoint> keypoints; 
        cv::Mat descriptors; 
        unsigned int detected_features = 0; 

        detector->detect(current_frame->imgLeft, keypoints, cv::noArray()); 
        descriptorExtractor->compute(current_frame->imgLeft, keypoints, descriptors); 

        for(unsigned int i = 0; i < keypoints.size(); i++)
        {
            current_frame->featuresFromLeftImg.emplace_back(new Feature{current_frame, keypoints[i], descriptors.row(i)}); //? have to check that 
            detected_features++; 
        }

        return detected_features;
    }

    unsigned int Tracking::findCorrespondingStereoFeatures()
    {
        //!DONE
        /*
            find in rigth image features corresponding to features detected in left img using LK optical flow 
        */
        std::vector<cv::Point2f> keypoints_left, keypoints_right; 
        for(auto &point :current_frame->featuresFromLeftImg)
        {
            keypoints_left.emplace_back(point->featurePoint_position.pt); 
            auto ptr_to_mappoint = point->map_point.lock(); 

            if(ptr_to_mappoint) //! i think that comparing with !=nullptr was a mistake 
            {
                //if observed point already exist in map then project this point from world to image and use it as initial guess 
                auto projected_point = camera_right->world2pixel(ptr_to_mappoint->position, current_frame->getFramePose()); 
                keypoints_right.emplace_back(projected_point[0], projected_point[1]); 
            } 
            else
            {
                //if map doesn't exit (initialization) that use the same pixel as in left image 
                keypoints_right.emplace_back(point->featurePoint_position.pt);
            } 
        }

        std::vector<uchar> status; // output status vector (of unsigned chars); each element of the vector is set to 1 if the flow for the corresponding features has been found, otherwise, it is set to 0.
        cv::Mat err; 
        cv::calcOpticalFlowPyrLK(current_frame->imgLeft, current_frame->imgRight, keypoints_left, keypoints_right, status, err, cv::Size(11,11), 3, 
                                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01), cv::OPTFLOW_USE_INITIAL_FLOW); 
        unsigned int foundCorrespondences = 0; 
        for(unsigned int i = 0; i < status.size(); i++)
        {
            if(status[i] == 1)
            {
                cv::KeyPoint keypoint_in_right(keypoints_right[i], 7); // keypoint pos and size 
                std::shared_ptr<Feature> new_feature(new Feature(current_frame, keypoint_in_right)); 
                new_feature->on_leftImg = false; 
                current_frame->featuresFromRightImg.emplace_back(new_feature); 
                foundCorrespondences++; 
            }
            else
            {
                current_frame->featuresFromRightImg.emplace_back(nullptr); 
            }
        }
        return foundCorrespondences; 
    }

    void Tracking::buildMap()
    {
        //!DONE
        /* 
            create initial map 
        */
        std::cout << "building initial map \n"; 

        unsigned int number_of_points_in_map = 0; 
        const unsigned int features_size = current_frame->featuresFromLeftImg.size(); 

        for(std::size_t i = 0; i < features_size; i++)
        {
            if (current_frame->featuresFromRightImg[i] == nullptr) 
                continue; 
            
            std::array<Eigen::Vector3d,2 > left_right_featurePoints_in_camera; // detected keypoints in camera coordinate system for triangulation
            //convert feature points to camera coordinate system
            left_right_featurePoints_in_camera.at(0) = (camera_left->pixel2camera(current_frame->featuresFromLeftImg[i]->featurePoint_position, 1)); 
            left_right_featurePoints_in_camera.at(1) = (camera_left->pixel2camera(current_frame->featuresFromRightImg[i]->featurePoint_position, 1)); 
            Eigen::Vector3d point_in_3D = Eigen::Vector3d::Zero(); 

            bool triSuccess = triangulate(left_right_featurePoints_in_camera, camera_left->Rt, camera_right->Rt, point_in_3D); //triangulate features to get point in 3d

            if((triSuccess == true) && (point_in_3D[2] > 0)) // check if Z is greater than O to eliminate points "behind" camera 
            {
                std::shared_ptr<MapPoint> new_mappoint(new MapPoint(MapPoint::mappoint_counter, point_in_3D)); //created new map point object 
                //MapPoint::mappoint_counter++; //! counter addition is in constructor
                number_of_points_in_map++; 
                std::cout << "mappoint id = " << new_mappoint->id << "\n";
                std::cout << "position of point = " << new_mappoint->getPosition() << "\n"; 

                new_mappoint->addFeature(current_frame->featuresFromLeftImg[i]); 
                new_mappoint->addFeature(current_frame->featuresFromRightImg[i]); 

                current_frame->featuresFromLeftImg[i]->map_point = new_mappoint; 
                current_frame->featuresFromRightImg[i]->map_point = new_mappoint; 

                map->insertPoint(new_mappoint);
            }
        }
        std::cout << "Map created with " << number_of_points_in_map << " points in map \n"; 

        current_frame->SetFrameToKeyframe(); 
        map->insertKeyFrame(current_frame); 
    }


    void Tracking::track()
    {
        //? in work 
        /*
        main tracking function 
        */

        if(prev_frame!=nullptr){
            //if at least 2 frames exist set current frame pose by multipling transMatrix with pose matrix (homogenous)
            current_frame->SetFramePose(transformationMatrix * prev_frame->getSophusFramePose());
        } else {
            std::cerr << "tracking needs two frames, prev_frame doesn't exist \n";  //!have to add smth to happen if there is not prev frame but i don't thing it's even possible  
        } 

        std::vector<cv::Point2f> keypoints_prev_frame, keypoints_current_frame; 

        // keypoints from previous keyframe/frame and initial guesses for current frame 
        for(auto &feature : prev_frame->featuresFromLeftImg )
        {
            if(feature->map_point.lock())
            {
                auto point_in_world = feature->map_point.lock(); 
                auto point_in_image = camera_left->world2pixel(point_in_world->position, current_frame->getFramePose()); 

                keypoints_prev_frame.emplace_back(feature->featurePoint_position.pt); 
                keypoints_current_frame.emplace_back(point_in_image[0], point_in_image[1]);  // transpose observed mappoint from previosu point to current frame img coordinates
            }
            else 
            {
                keypoints_prev_frame.emplace_back(feature->featurePoint_position.pt); 
                keypoints_current_frame.emplace_back(feature->featurePoint_position.pt); 
            }   
        }

        std::vector<uchar> status;
        cv::Mat err;
        cv::calcOpticalFlowPyrLK(prev_frame->imgLeft, current_frame->imgLeft, keypoints_prev_frame, keypoints_current_frame, status, err, cv::Size(11,11), 3, 
                                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01), cv::OPTFLOW_USE_INITIAL_FLOW); 

        unsigned int number_of_matched_points = 0; 

        for(unsigned int i = 0; i < status.size();i++ )
        {
            if(status[i] == 1)
            {
                cv::KeyPoint current_frame_new_kp(keypoints_current_frame[i], 7); // keypoint pos and size 
                std::shared_ptr<Feature> new_feature(new Feature(current_frame, current_frame_new_kp)); 
                new_feature->map_point = prev_frame->featuresFromLeftImg[i]->map_point; 

                current_frame->featuresFromLeftImg.emplace_back(new_feature);
                number_of_matched_points++;
            }
        }

        std::cout << "number of matched/tracked points = " << number_of_matched_points << "\n"; 

        unsigned int num_of_inliers = estimatePose(); 
        std::cout << "Number of inliers " << num_of_inliers << "\n"; 

        //! decision if current frame is new keyframe 
        if( num_of_inliers < num_of_features_for_keyframe ) // work on this 
        {
            //
            newKeyframeInsertion(); 
        }
        
        transformationMatrix = (current_frame->getSophusFramePose() * prev_frame->getSophusFramePose().inverse()); 
       
        visualizer->addNewFrame(current_frame); 
    }

    void Tracking::newKeyframeInsertion()
    {
        //! DONE
        /*
        make current frame a keyframe, insert keyframe to map and visualizer, add 
        */
        current_frame->SetFrameToKeyframe(); 
        std::cout << "frame with id" << current_frame->id << " is new keyframe id = " << current_frame->keyframe_id << "\n"; 
        map->insertKeyFrame(current_frame); 

        // add features from keyframe to observed mappoints
        for (auto &feature : current_frame->featuresFromLeftImg) 
        {
            auto mappoint = feature->map_point.lock();
            if(mappoint)
            {
                mappoint->addFeature(feature);
            }  
        }

        unsigned int num_of_feature =  detectFeatures();
        std::cout << "found " << num_of_feature << "new features \n"; 
        findCorrespondingStereoFeatures();
        createNewMapPoints(); 

        l_mapping->updateMap(); 
        visualizer->getMapUpdate(); 

        std::cout << " added keyframe \n"; 
    }

    void Tracking::createNewMapPoints()
    {
        //!DONE
        /*
        triangulate new map points //basiclly the same stuff as in buildMap //?maybe use this function in buildMap()?
        */

       Eigen::Matrix4d currentPose_Twc = current_frame->getFramePose().inverse(); //camera to world transformation matrix 
       unsigned int number_of_triangulatedPoints = 0; 
       

       for(unsigned int i  = 0; i < current_frame->featuresFromLeftImg.size(); i++)
       {
        // check if feature has any owners https://en.cppreference.com/w/cpp/memory/weak_ptr/expired and if feature from right were found 
            if((current_frame->featuresFromLeftImg[i]->map_point.expired()) && (current_frame->featuresFromRightImg[i] != nullptr))   
            {
                std::array<Eigen::Vector3d, 2> left_right_featurePoints_in_camera;
                left_right_featurePoints_in_camera.at(0) = (camera_left->pixel2camera(current_frame->featuresFromLeftImg[i]->featurePoint_position, 1)); 
                left_right_featurePoints_in_camera.at(1) = (camera_left->pixel2camera(current_frame->featuresFromRightImg[i]->featurePoint_position, 1)); 
                Eigen::Vector3d point_in_3D = Eigen::Vector3d::Zero();

                bool triSuccess = triangulate(left_right_featurePoints_in_camera, camera_left->Rt, camera_right->Rt, point_in_3D); //triangulate features to get point in 3d

                if((triSuccess==true) && (point_in_3D[2] > 0) ) // check if Z is greater than O to eliminate points "behind" camera 
                {
                    point_in_3D = currentPose_Twc.block<3,3>(0,0) * point_in_3D; 

                    auto new_mappoint = std::shared_ptr<MapPoint>(new MapPoint(MapPoint::mappoint_counter, point_in_3D)); //created new map point object 
                    //MapPoint::mappoint_counter++; // mappoint id counter 
                    number_of_triangulatedPoints++; 

                    new_mappoint->addFeature(current_frame->featuresFromLeftImg[i]); 
                    new_mappoint->addFeature(current_frame->featuresFromRightImg[i]); 

                    current_frame->featuresFromLeftImg[i]->map_point = new_mappoint; 
                    current_frame->featuresFromRightImg[i]->map_point = new_mappoint; 

                    map->insertPoint(new_mappoint);
                }
            }
       }

       std::cout << "triangulated : " << number_of_triangulatedPoints << " new points \n"; 
    }


    unsigned int Tracking::estimatePose()
    {
        //!DONE 
        unsigned int good_points = 0; //inliers 
        unsigned int bad_points = 0; 

        //https://www.wangxinliu.com/slam/optimization/research&study/g2o-1/
        //? https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/tutorial_slam2d/tutorial_slam2d.cpp
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; 

        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>())); 

        g2o::SparseOptimizer optimizer; 
        optimizer.setAlgorithm(solver); 
        optimizer.setVerbose(false); 

        //add first pose vertex 
        Pose3DVertex* pose_vertex = new Pose3DVertex(); 
        pose_vertex->setId(0); 
        pose_vertex->setEstimate(current_frame->getSophusFramePose()); 
        pose_vertex->setFixed(false); 
        optimizer.addVertex(pose_vertex); //add vertex to graph 

        //create graph 
        std::vector<PoseEdge*> pose_edges; 
        std::vector<std::shared_ptr<Feature>> features; 

        auto K = camera_left->K_eigen; 
        unsigned int edge_id = 1; 

        std::cout << "number of features in current frame = " << current_frame->featuresFromLeftImg.size() << "\n"; 

        for(unsigned int i = 0; i < current_frame->featuresFromLeftImg.size(); i++)
        {
            auto map_point = current_frame->featuresFromLeftImg[i]->map_point.lock(); 

            if(map_point)
            {
                features.push_back(current_frame->featuresFromLeftImg[i]); 
                PoseEdge* edge = new PoseEdge(map_point->getPosition(), K); 
                edge->setId(edge_id);
                edge->setVertex(0,pose_vertex); 

                auto pointxy = current_frame->featuresFromLeftImg[i]->featurePoint_position.pt; 
                Eigen::Vector2d measurment(pointxy.x, pointxy.y);
                edge->setMeasurement(measurment); 

                edge->setInformation(Eigen::Matrix2d::Identity()); 
                edge->setRobustKernel(new g2o::RobustKernelHuber);

                pose_edges.push_back(edge); 
                optimizer.addEdge(edge);
                edge_id++; 
            }
        }
        std::cout << "size of pose edges " << pose_edges.size() << "\n"; 

        //chi squared outlier detection 
        for (int j = 0; j < 4; j++) 
        {
            pose_vertex->setEstimate(current_frame->getSophusFramePose());
            optimizer.initializeOptimization(0);
            optimizer.optimize(10);
            bad_points = 0;

            // count the outliers
            for (size_t i = 0; i < pose_edges.size(); i++) 
            {
                auto edge = pose_edges[i];
                if (features[i]->outlier == true) 
                {
                    edge->computeError();
                }

                const float chi2 = edge->chi2(); 
                std::cout << "output chi2 = " << chi2 << "\n";

                if (chi2 > chi_squared_treshold) 
                {
                    features[i]->outlier = true;
                    edge->setLevel(1);
                    bad_points++;
                } else {
                    features[i]->outlier = false;
                    edge->setLevel(0);
                };

                if (j == 2) {
                    edge->setRobustKernel(nullptr);
                }
            }
            if(optimizer.edges().size() < 5)
                break; 
        }

        std::cout << "detected " << bad_points << "outliers from " << features.size() << "feature points \n"; 
        good_points = features.size() - bad_points;  

        current_frame->SetFramePose(pose_vertex->estimate()); 

        for(auto &feature : features)
        {
            if(feature->outlier == true)
            {
                feature->map_point.reset();  //Releases the reference to the managed object https://en.cppreference.com/w/cpp/memory/weak_ptr/reset
                //feature->outlier = false; 
            }
        }

        return good_points; 
    } 

    void Tracking::restartTracking()
    {
        
    }
}