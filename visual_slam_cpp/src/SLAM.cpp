#include "../include/SLAM.hpp"
#include "../include/FrameExtraction.hpp"

namespace mrVSLAM
{
    //SLAM Class constructors 
    SLAM::~SLAM() = default;  
    SLAM::SLAM(std::string sequence_number) 
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
    }

//###################

    int SLAM::runMonoSLAM()
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

            Frame frame(img, camera.K, frame_counter); // create Frame object that holds all information about current frame/img 

            
 
            //////// ----- Algorithm End ----- //////////

            
            loopEnd = cv::getTickCount();
            fps = 1/((loopEnd - loopStart)/cv::getTickFrequency()); 
            auto cend = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cend - begin);

            // cv::drawKeypoints(frame, features.frame_keypoints, frame, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);
        
            for(int p = 0; p < features.matched_keypoints.size(); p++)
            { 
                cv::circle(img, features.matched_keypoints[p][0], 3, cv::Scalar(0,255,0));
                cv::line(img, features.matched_keypoints[p][1], features.matched_keypoints[p][0], cv::Scalar(255,0,0), 1); 
            }

            features.matched_keypoints.clear(); 
            features.frame_keypoints.clear(); 

            cv::putText(img, "fps: " + std::to_string(fps), 
                        cv::Point(30,50), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0,0,255),2);
            cv::imshow("Camera Img", img);
            std::cout << "Frame num: " << frame_counter++ << "\n" << "time: " << elapsed.count() << "\n"; 

            char key = (char)cv::waitKey(33); 
            if(key == 'q' || key == 27)
                break;
        }
        sequence.release(); 
        cv::destroyAllWindows(); 
        return 0; 
    }

    int SLAM::runStereoSLAM()
    {
        cv::Mat imgLeft(370, 1226,CV_8UC1);
        cv::Mat imgRight(370, 1226,CV_8UC1);

        cv::VideoCapture sequenceLeft;
        cv::VideoCapture sequenceRight; 
        sequenceLeft.open(dataset.left_imgs_path, cv::CAP_IMAGES); 
        sequenceRight.open(dataset.right_imgs_path, cv::CAP_IMAGES);

        if (!sequenceLeft.isOpened() || !sequenceRight.isOpened()) {
            std::cerr << "Failed to open Image Sequence!\n"; 
            return -1;
        }

        while (true)
        {
            /* code */
        }
        
    }   


    void SLAM::showResult()
    {
        plotPoses(dataset.ground_truth_poses, frame_counter); 

    }
}