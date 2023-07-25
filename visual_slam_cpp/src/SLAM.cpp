#include "../include/SLAM.hpp"
#include "../include/Stereo.hpp"

namespace mrVSLAM
{
    //SLAM Class constructors 
    SLAM::~SLAM() = default;  
    SLAM::SLAM(std::string sequence_number) noexcept
    {
        // get kitti dataset 
        dataset.chooseSequence(sequence_number); 
        dataset.readCalibData(); // get camera calibration matrix 
        dataset.showPmatricies(); 
        dataset.getGTposes(); // get ground truth trajectory 
        // std::cout << std::setprecision(1) << std::fixed << dataset.ground_truth_poses[0] <<  "\n";  

        camera.setCamera(dataset.P0); // set left camera
        right_camera.setCamera(dataset.P1);  // set right camera 
        baseline = getStereoBaseline(camera.t, right_camera.t); // calculate baseline 
        std::cout << "baseline = " << baseline << "\n"; 
        std::cout << "\n" << "-------------"<<"\n"; 

        frames.reserve(6500*sizeof(Frame));
        performance.reserve(2000); 
    }

//###################

    int SLAM::runMonoSLAM() noexcept
    {
        cv::Mat img(370, 1226,CV_8UC1); // declare img size and type, super important 

        // Create img sequence and get 
        cv::VideoCapture sequence; 
        sequence.open(dataset.left_imgs_path, cv::CAP_IMAGES);

        cv::namedWindow("Camera Img", cv::WINDOW_AUTOSIZE); 

        if (!sequence.isOpened()) {
            std::cerr << "Failed to open Image Sequence!\n"; 
            return -1;
        }

        while(true)
        {
            sequence.read(img);
            
            if(img.empty()) {
                std::cout << "End of sequance \n"; 
                break;
            }

            loopStart = cv::getTickCount(); 
            auto begin = std::chrono::high_resolution_clock::now();

            //////// ----- Algorithm body ------ ///////// 

            // Frame frame(img, camera.K, frame_counter, 500); // create Frame object that holds all information about current frame/img 
            // frames.emplace_back(frame); 
            frames.emplace_back(img, camera.K, frame_counter, 500); 
 
            //////// ----- Algorithm End ----- //////////

            
            loopEnd = cv::getTickCount();
            fps = 1/((loopEnd - loopStart)/cv::getTickFrequency()); 
            auto cend = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cend - begin);
            performance.emplace_back(fps); 

            cv::drawKeypoints(img, frames.back().frameFeaturePoints, img, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);
        
            // for(int p = 0; p < features.matched_keypoints.size(); p++)
            // { 
            //     cv::circle(img, features.matched_keypoints[p][0], 3, cv::Scalar(0,255,0));
            //     cv::line(img, features.matched_keypoints[p][1], features.matched_keypoints[p][0], cv::Scalar(255,0,0), 1); 
            // }

            // features.matched_keypoints.clear(); 
            // features.frame_keypoints.clear(); 

            cv::putText(img, "fps: " + std::to_string(fps), 
                        cv::Point(30,50), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0,0,255),2);
            cv::imshow("Camera Img", img);
            std::cout << "Frame num: " << frame_counter++ << "\n" << "time: " << elapsed.count() << "\n"; 

            char key = (char)cv::waitKey(1); 
            if(key == 'q' || key == 27)
                break;
        }
        sequence.release(); 
        cv::destroyAllWindows(); 
        return 0; 
    }

    int SLAM::runStereoSLAM() noexcept
    {
        cv::Mat imgLeft(370, 1226,CV_8UC1);
        cv::Mat imgRight(370, 1226,CV_8UC1);

        cv::VideoCapture sequenceLeft, sequenceRight;
        sequenceLeft.open(dataset.left_imgs_path, cv::CAP_IMAGES); 
        sequenceRight.open(dataset.right_imgs_path, cv::CAP_IMAGES);

        if (!sequenceLeft.isOpened() || !sequenceRight.isOpened()) {
            std::cerr << "Failed to open Image Sequence!\n"; 
            return -1;
        }

        StereoDepth stereo(stereoMatcherType::BlockMatching, baseline, camera.fx); 


        while (true)
        {
            sequenceLeft.read(imgLeft); 
            sequenceRight.read(imgRight); 

            if(imgLeft.empty() || imgRight.empty()) {
                std::cout << "End of sequance \n"; 
                break;
            }
            loopStart = cv::getTickCount();


            stereo.calculateDepth(imgLeft, imgRight); 


            loopEnd = cv::getTickCount();
            fps = 1/((loopEnd - loopStart)/cv::getTickFrequency()); 
            cv::putText(stereo.disparityMap, "fps: " + std::to_string(fps), 
                        cv::Point(30,50), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0,0,255),2);
            cv::imshow("Camera Img", stereo.disparityMap);

            char key = (char)cv::waitKey(33); 
            if(key == 'q' || key == 27)
                break;
        }
        sequenceLeft.release(); 
        sequenceRight.release();
        cv::destroyAllWindows(); 
        return 0; 
    }   
}