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

        frames.reserve(500+dataset.ground_truth_poses.size());
        performance.reserve(2000); 
    }

//###################

    int SLAM::runMonoSLAM() noexcept
    {
        cv::Mat img(370, 1226, CV_8UC1); // declare img size and type, super important 

        // Create img sequence and get 
        cv::VideoCapture sequence; 
        sequence.open(dataset.left_imgs_path, cv::CAP_IMAGES);

        cv::namedWindow("Camera Img", cv::WINDOW_AUTOSIZE); 

        if (!sequence.isOpened()) {
            std::cerr << "Failed to open Image Sequence!\n"; 
            return -1;
        }

        FrameMatcher matcher(MatcherType::BruteForce); 

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

            frames.emplace_back(img, camera.K, frame_counter, 500); // create Frame object and emplace it in frames vector
            if(frames.size() <= 1)
                continue;
            matcher.matchFrames(frames.end()[-1], frames.end()[-2], 0.7f); // get matches //https://stackoverflow.com/questions/44831793/what-is-the-difference-between-vector-back-and-vector-end
            
            getRelativeFramePose(); 
            
            //////// ----- Algorithm End ----- //////////

            std::cout << "frame id = " <<frames.end()[-1].frameId << "\n"; 

            
            loopEnd = cv::getTickCount();
            fps = 1/((loopEnd - loopStart)/cv::getTickFrequency()); 
            auto cend = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cend - begin);
            performance.emplace_back(fps); 

            //cv::drawKeypoints(img, frames.back().frameFeaturePoints, img, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);
        
            for(int p = 0; p < matcher.matchedKeypoints.size(); p++)
            { 
                cv::circle(img, matcher.matchedKeypoints[p][0], 3, cv::Scalar(255,255,0));
                cv::line(img, matcher.matchedKeypoints[p][1], matcher.matchedKeypoints[p][0], cv::Scalar(255,0,0), 1); 
            }
            
            matcher.matchedKeypoints.clear(); 

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


     void SLAM::getRelativeFramePose(std::vector<cv::Point2f> &frame1points, std::vector<cv::Point2f> &frame2points)
     {
        cv::Mat essentialMatrix; 

        essentialMatrix = cv::findEssentialMat(frame1points, frame2points, camera.K, cv::RANSAC, 0.99, 1.0, 100, cv::noArray()); 
        //https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?highlight=decomposeessentialmat#void%20decomposeEssentialMat(InputArray%20E,%20OutputArray%20R1,%20OutputArray%20R2,%20OutputArray%20t)
        //cv::decomposeEssentialMat()

        cv::recoverPose(essentialMatrix, frame1points, frame2points, camera.K, R, t); 

     }

}