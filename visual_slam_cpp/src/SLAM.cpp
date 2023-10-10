#include "../include/SLAM.hpp"

namespace mrVSLAM
{
    StereoSLAM::StereoSLAM(std::string sequence_number)
    {
        //* Dataset initialization 
        dataset = std::shared_ptr<KITTI_Dataset>(new KITTI_Dataset);
        dataset->chooseSequence(sequence_number); 
        dataset->readCalibData(); 
        dataset->showPmatricies(); 
        dataset->getGTposes();

        camera_left = std::shared_ptr<Camera>(new Camera);
        camera_right = std::shared_ptr<Camera>(new Camera);

        // camera/s setup 
        camera_left->setCamera(dataset->P0); 
        camera_right->setCamera(dataset->P1); 
        camera_left->baseline = 0; 
        camera_right->baseline = getStereoBaseline(camera_left->t, camera_right->t); 

        // setup of main components 
        map = std::shared_ptr<Map>(new Map); 
        visualizer = std::shared_ptr<Visualizer>(new Visualizer); 
        local_mapping = std::shared_ptr<LocalMapping>(new LocalMapping); 
        tracking = std::shared_ptr<Tracking>(new Tracking(DETECTOR::GFTT)); 

        tracking->setTracking(map, visualizer, local_mapping, camera_left, camera_right); // setup visualizer 
        visualizer->setVisualizer(map, camera_left->K_eigen); 
        local_mapping->setLocalMapping(map, camera_left, camera_right); 
    }


    int StereoSLAM::Run()
    {
        // Create img sequence and get 
        std::cout << "Running main thread \n";
        cv::VideoCapture sequenceLeft; 
        cv::VideoCapture sequenceRight; 
        sequenceLeft.open(dataset->left_imgs_path, cv::CAP_IMAGES);
        sequenceRight.open(dataset->right_imgs_path, cv::CAP_IMAGES);

        if (!sequenceLeft.isOpened() || !sequenceRight.isOpened()) {
            std::cerr << "Failed to open Image Sequence!\n"; 
            return -1;
        }

        while(true)
        {
            sequenceLeft.read(imgLeft);
            sequenceRight.read(imgRight);

            if(imgLeft.empty()) {
                std::cout << "End of sequance \n"; 
                break;
            }

            // loopStart = cv::getTickCount(); 
            auto beginT = std::chrono::high_resolution_clock::now();
            // #####################
            Sophus::SE3d initial_matrix; // sophus matiris is identity matrix by default
            
            //* create Frame object and pointer to it
            std::shared_ptr<Frame> frame(new Frame(frame_counter, initial_matrix, imgLeft, imgRight)); //? add K to frame class

            //* pass frame to tracking and run tracking 
            bool tracking_working = tracking->addFrameAndTrackStereo(frame); 

            frame_counter++; 
            std::cout << "frame counter " << frame_counter << "\n";

            //############
            // loopEnd = cv::getTickCount();
            // fps = 1/((loopEnd - loopStart)/cv::getTickFrequency()); 
            auto endT = std::chrono::high_resolution_clock::now();
            auto elapsedT = std::chrono::duration_cast<std::chrono::milliseconds>(endT - beginT);
            // performance.emplace_back(fps); 
            std::cout << "Loop time is " << elapsedT.count() << "ms \n"; 

        }
        
        sequenceLeft.release(); 
        sequenceRight.release();
    
        visualizer->closeVisualizer(); 
        cv::destroyAllWindows(); 
        return 0; 
    }
}