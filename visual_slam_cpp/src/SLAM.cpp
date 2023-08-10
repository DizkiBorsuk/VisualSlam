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
        trajectory.reserve(500+dataset.ground_truth_poses.size()); 

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

        FeatureExtractor featureExtractor(ExtractorType::ORB, 800); 
        FeatureExtractor *extractorPointer = &featureExtractor; 
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


            frames.emplace_back(img, camera.K, frame_counter, 800, extractorPointer); // create Frame object and emplace it in frames vector
            if(frames.size() <= 1)
                continue;
            matcher.matchFrames(frames.end()[-1], frames.end()[-2], 0.7f); // get matches //https://stackoverflow.com/questions/44831793/what-is-the-difference-between-vector-back-and-vector-end
            getRelativeFramePose(matcher.matchedKeypoints[0],matcher.matchedKeypoints[1], frames.end()[-1].pose); 
            
            //////// ----- Algorithm End ----- /////////
            
            loopEnd = cv::getTickCount();
            fps = 1/((loopEnd - loopStart)/cv::getTickFrequency()); 
            auto cend = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cend - begin);
            performance.emplace_back(fps); 

            //cv::drawKeypoints(img, frames.back().frameFeaturePoints, img, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);
                
            // for(int p = 0; p < matcher.matchedKeypoints[0].size(); p++)
            // { 
            //     cv::circle(img, matcher.matchedKeypoints[0][p], 3, cv::Scalar(255,255,0));
            //     cv::line(img, matcher.matchedKeypoints[1][p], matcher.matchedKeypoints[0][p], cv::Scalar(0,0,0), 1); 
            // }
            
            matcher.matchedKeypoints[0].clear(); // always have to clear them
            matcher.matchedKeypoints[1].clear(); 

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


    void SLAM::getRelativeFramePose(const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2, 
                                    cv::Matx44d &pose)
    {
        cv::Matx33d R;
        cv::Matx31d t; 
        cv::Matx34d Rt; 
        cv::Mat mask;  

        essentialMatrix = cv::findEssentialMat(points1, points2, camera.K, cv::RANSAC, 0.99, 1.0, 100, mask); 
        //std::cout << "Essential matrix = \n" << essentialMatrix << "\n matrix size = " << essentialMatrix.size() <<"\n";
        //https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        cv::recoverPose(essentialMatrix, points1, points2, camera.K, R, t, mask); 

        cv::hconcat(R, t, Rt);  
        std::cout << "pose = \n" << Rt << "\n"; 
        //std::cout << "poza = " << pose <<"\n"; 
    }

}