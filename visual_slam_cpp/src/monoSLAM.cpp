#include "../include/monoSLAM.hpp"
#include "../include/FeatureExtraction.hpp"
#include "../include/camera.hpp"

namespace mrVSLAM
{
    void monoSLAM::poseEstimationEpiCons(std::vector<std::vector<cv::Point2f>> &matched_points)
    {   
        std::vector<cv::Point2f> points_frame1, points_frame2; 

        std::cout << "mathced points size = " << matched_points.size() << "\n"; 
        if(matched_points.size() >= 8)
        {
            for(int i = 0; i < matched_points.size(); i++)
            {
                points_frame1.emplace_back(matched_points[i][0]); 
                points_frame2.emplace_back(matched_points[i][1]); 
            }

            F = cv::findFundamentalMat(points_frame1, points_frame2, cv::FM_8POINT); 
            std::cout << "F = " << F << "\n"; 
            E = cv::findEssentialMat(points_frame1, points_frame2, fy, principialPoint); 
            H = cv::findHomography(points_frame1, points_frame2, cv::RANSAC, 3); 
            cv::recoverPose(E, points_frame1, points_frame2, R, t, fy, principialPoint);
            std::cout << "R = " << R <<"\n t = " << t << "\n";
            cv::cv2eigen(R,R_e); 
            cv::cv2eigen(t, t_e); 
            
            Rt << R_e, t_e; 
            
            std::cout << "Rt = " << Rt <<"\n";
            poses.emplace_back(Rt); 
        }
  
    }

    int mrVSLAM::monoSLAM::executeMonoSLAM(const std::string& imgs_path)
    {
        //Class objects 
        mrVSLAM::FeatureExtraction features(ExtractorType::orb_gptt, false, 500);
    
        // Variables 
        cv::Mat frame(370, 1226,CV_8UC1); // declare img size and type, super important 
        int start, end, framesPerSecond; 

        // Create img sequence and get 
        cv::VideoCapture sequence; 
        sequence.open(imgs_path, cv::CAP_IMAGES);

        cv::namedWindow("Camera Img", cv::WINDOW_AUTOSIZE); 

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
        
            features.getFeatures(frame); 
            features.matchFeaturesBF(0.75f); 
            //features.matchFeaturesBF(0.75f);  
            //poseEstimationEpiCons(features.matched_keypoints); 
            
 ; 
            //////// ----- Algorithm End ----- //////////

            
            end = cv::getTickCount();
            auto cend = std::chrono::high_resolution_clock::now();
            framesPerSecond = 1/((end - start)/cv::getTickFrequency()); 
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cend - begin);

            // cv::drawKeypoints(frame, features.frame_keypoints, frame, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);
        
            for(int p = 0; p < features.matched_keypoints.size(); p++)
            { 
                cv::circle(frame, features.matched_keypoints[p][0], 3, cv::Scalar(0,255,0));
                cv::line(frame, features.matched_keypoints[p][1], features.matched_keypoints[p][0], cv::Scalar(255,0,0), 1); 
            }

            features.matched_keypoints.clear(); 
            features.frame_keypoints.clear(); 

            cv::putText(frame, "fps: " + std::to_string(framesPerSecond), 
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
}

