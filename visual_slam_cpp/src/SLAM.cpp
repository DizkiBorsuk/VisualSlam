#include "../include/SLAM.hpp"

        

namespace mrVSLAM
{
    unsigned int Frame::keyframe_counter = 0; // well, that's stupid but i don't have better idea 

    StereoDirectSLAM::StereoDirectSLAM(std::string sequence_number)
    {
        //* Dataset initialization 
        dataset = std::shared_ptr<KITTI_Dataset>(new KITTI_Dataset);
        dataset->chooseSequence(sequence_number); 
        dataset->readCalibData(); 
        dataset->showPmatricies(); 
        dataset->getGTposes();

        // camera/s setup 
        camera_left.setCamera(dataset->P0); 
        camera_right.setCamera(dataset->P1); 
        camera_left.baseline = 0; 
        camera_right.baseline = getStereoBaseline(camera_left.t, camera_right.t); 

        auto ptr_to_camera_left = std::shared_ptr<Camera>(&camera_left); 
        auto ptr_to_camera_right = std::shared_ptr<Camera>(&camera_right); 

        // setup of main components 
        map = std::shared_ptr<Map>(new Map); 
        visualizer = std::shared_ptr<Visualizer>(new Visualizer); 
        backend = std::shared_ptr<Backend>(new Backend); 
        tracking = std::shared_ptr<Tracking>(new Tracking); 

        tracking->setTracking(map, visualizer, backend); // setup visualizer 
        visualizer->setMapPtr(map); 
    }


    int StereoDirectSLAM::Run()
    {
        // Create img sequence and get 
        cv::VideoCapture sequenceLeft; 
        cv::VideoCapture sequenceRight; 
        sequenceLeft.open(dataset->left_imgs_path, cv::CAP_IMAGES);
        sequenceRight.open(dataset->right_imgs_path, cv::CAP_IMAGES);

        cv::namedWindow("Camera Img", cv::WINDOW_AUTOSIZE); 

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

            loopStart = cv::getTickCount(); 
            auto begin = std::chrono::high_resolution_clock::now();
            // #####################
            cv::Matx44d eye_matrix = cv::Matx44d::eye(); //! temp solution, change later to camera R matrix  
            
            //* create Frame object and pointer to it
            std::shared_ptr<Frame> frame = std::shared_ptr<Frame>(new Frame(frame_counter, eye_matrix, imgLeft, imgRight)); //? add K to frame class

            //* pass frame to tracking and run tracking 
            tracking->addFrameAndTrack(frame); 

            


            frame_counter++; 

            //############
            loopEnd = cv::getTickCount();
            fps = 1/((loopEnd - loopStart)/cv::getTickFrequency()); 
            auto cend = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cend - begin);
            performance.emplace_back(fps); 

        }
        sequenceLeft.release(); 
        sequenceRight.release();
    
        visualizer->closeVisualizer(); 
        cv::destroyAllWindows(); 
        return 0; 
    }
}