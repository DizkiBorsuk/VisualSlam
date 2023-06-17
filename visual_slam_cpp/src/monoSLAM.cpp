#include "../include/monoSLAM.hpp"
#include "../include/FeatureExtraction.hpp"

namespace mrVSLAM
{
    monoSLAM::monoSLAM(const Eigen::Matrix<double,3,4> &projectionMatrix) noexcept
    {
        //decomposing projection matrix P to intrinsic/camera matrix K,
        K = projectionMatrix.block<3,3>(0,0); 
        cx = projectionMatrix.coeff(0,2); 
        cy = projectionMatrix.coeff(1,2); 
        fx = projectionMatrix.coeff(0,0); 
        fy = projectionMatrix.coeff(1,1); 

    }

    void monoSLAM::poseEstimationEpiCons(std::vector<std::vector<cv::Point2f>> &matched_points, std::vector<cv::DMatch> &matches)
    {   
        // cv::Mat cvF(3,3, CV_8UC1); 
        // cv::Mat cvE(3,3, CV_8UC1); 
        // cv::Mat cvH(3,3, CV_8UC1);
        // cv::Mat cvR(3,3, CV_8UC1);
        // cv::Mat cvt; 

        // std::vector<cv::Point2f> points_frame1, points_frame2; 

        // for(auto points: matched_points)
        // {
        //     points_frame1.emplace_back(points[0]); 
        //     points_frame1.emplace_back(points[1]); 
        // }

        // cvF = cv::findFundamentalMat(points_frame1, points_frame2, cv::FM_8POINT); 
        // std::cout << "F = " << cvF << "\n"; 
        // cvE = cv::findEssentialMat(points_frame1, points_frame2, fy, (cx, cy)); 
        // cvH = cv::findHomography(points_frame1, points_frame2, cv::RANSAC, 3); 
        // cv::recoverPose(cvE, points_frame1, points_frame2, cvR, cvt, fy, (cx,cy)); 
    }

    int mrVSLAM::monoSLAM::executeMonoSLAM(const std::string& imgs_path)
    {
        //Class objects 
        mrVSLAM::FeatureExtraction features(ExtractorType::orb_fast, false, 500);
    
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
            //features.matchFeaturesBF(0.75f); 
            features.matchFeaturesFlann(0.6f); 

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

