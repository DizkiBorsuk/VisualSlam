#include "../include/SLAM.hpp"
#include "../include/FeatureExtraction.hpp"

int mrVSLAM::SLAM::executeMonoSLAM(std::string& imgs_path)
{
    //Class object 
    mrVSLAM::FeatureExtraction features; 

    cv::Mat frame; 
    int start, end; 
    double framesPerSecond; 

    cv::namedWindow("Camera Img", cv::WINDOW_AUTOSIZE); 

    cv::VideoCapture sequence; 
    sequence.open(imgs_path, cv::CAP_IMAGES);

    if (!sequence.isOpened())
    {
      std::cerr << "Failed to open Image Sequence!\n"; 
      return -1;
    }

    while(true)
    {
        sequence.read(frame);

        if(frame.empty())
        {
            std::cout << "End of sequance \n"; 
            break;
        }
        
        start = cv::getTickCount(); 
        auto begin = std::chrono::high_resolution_clock::now();

        //////// ----- Algorithm body ------ ///////// 
        features.num_features = 200; 
        features.getFeatures(frame, FeatureExtraction::desctiptor_T::orb); 
        features.matchFeaturesFlann( 0.6f); 


        //////// ----- Algorithm End ----- //////////

            
        //cv::hconcat(left_frame, right_frame, stereo); 
        
        end = cv::getTickCount();
        auto cend = std::chrono::high_resolution_clock::now();
        framesPerSecond = 1/((end - start)/cv::getTickFrequency()); 
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cend - begin);

        cv::drawKeypoints(frame, features.keypoints, frame, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        
        for(int p = 0; p < features.matched_keypoints.size(); p++)
        {
            cv::line(frame, features.matched_keypoints[p][1], features.matched_keypoints[p][0], cv::Scalar(255,0,0), 1); 
        }
        features.matched_keypoints.clear(); 

        cv::putText(frame, "fps: " + std::to_string(int(framesPerSecond)), 
                    cv::Point(30,50), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0,0,255),2);
        cv::imshow("Camera Img", frame);
        std::cout << "Frame num: " << f_counter++ << "\n" << "time: " << elapsed.count() << "\n"; 

        char key = (char)cv::waitKey(66); 
        if(key == 'q' || key == 27)
            break;
    }
    cv::destroyAllWindows(); 

    return 0; 
}


int mrVSLAM::SLAM::executeMonoSLAM(std::string& imgs_path, bool GPU)
{
        //Class object 
    mrVSLAM::FeatureExtraction features; 


    cv::Mat frame; 
    cv::cuda::GpuMat gpu_frame; 
    int start, end; 
    double framesPerSecond; 

    cv::namedWindow("Camera Img", cv::WINDOW_AUTOSIZE); 

    cv::VideoCapture sequence; 
    sequence.open(imgs_path, cv::CAP_IMAGES);

    if (!sequence.isOpened())
    {
      std::cerr << "Failed to open Image Sequence!\n"; 
      return -1;
    }

    while(true)
    {
        sequence.read(frame);

        if(frame.empty())
        {
            std::cout << "End of sequance \n"; 
            break;
        }
        
        //start = cv::getTickCount(); 
        auto begin = std::chrono::high_resolution_clock::now();
        gpu_frame.upload(frame); 
        

        //////// ----- Algorithm body ------ ///////// 
        features.num_features = 200; 
        features.getFeatures(frame, FeatureExtraction::desctiptor_T::orb); 
        features.matchFeaturesFlann( 0.6f); 


        //////// ----- Algorithm End ----- //////////

    

        
        //cv::hconcat(left_frame, right_frame, stereo); 
        
        //end = cv::getTickCount();
        auto cend = std::chrono::high_resolution_clock::now();
        //framesPerSecond = 1/((end - start)/cv::getTickFrequency()); 
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cend - begin);

        cv::drawKeypoints(frame, features.keypoints, frame, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        
        for(int p = 0; p < features.matched_keypoints.size(); p++)
        {
            cv::line(frame, features.matched_keypoints[p][1], features.matched_keypoints[p][0], cv::Scalar(255,0,0), 1); 
        }
        features.matched_keypoints.clear(); 

        // cv::putText(left_frame, "fps: " + std::to_string(int(framesPerSecond)), 
        //             cv::Point(30,50), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0,0,255),2);
        cv::imshow("Camera Img", frame);
        std::cout << "Frame num: " << f_counter++ << "\n" << "time: " << elapsed.count() << "\n"; 

        char key = (char)cv::waitKey(66); 
        if(key == 'q' || key == 27)
            break;
    }
    cv::destroyAllWindows(); 

    return 0; 
}

