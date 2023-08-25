#include "../include/StereoDirectSLAM.hpp"

        

namespace mrVSLAM
{
    unsigned int Frame::keyframe_counter = 0; // well, that's stupid but i don't have better idea 

    StereoDirectSLAM::StereoDirectSLAM(std::string sequence_number)
    {
        map = std::shared_ptr<Map>(new Map); 
        visualizer = std::shared_ptr<Visualizer>(new Visualizer); 
        backend = std::shared_ptr<Backend>(new Backend); 
        tracking = std::shared_ptr<Tracking>(new Tracking); 

        tracking->setTracking(map, visualizer, backend); // setup visualizer 
        
        //* Dataset initialization 
        dataset = std::shared_ptr<KITTI_Dataset>(new KITTI_Dataset);
        dataset->chooseSequence(sequence_number); 
        dataset->readCalibData(); 
        dataset->showPmatricies(); 
        dataset->getGTposes();
    }


    int StereoDirectSLAM::Run()
    {
        cv::Mat imgLeft(370, 1226, CV_8UC1); // declare img size and type, super important 
        cv::Mat imgRight(370, 1226, CV_8UC1);

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
            cv::Matx44d eye_matrix = cv::Matx44d::eye(); //! temp solution, change later to camera R matrix  
            
            //* create Frame object and pointer to it
            std::shared_ptr<Frame> frame = std::shared_ptr<Frame>(new Frame(frame_counter, eye_matrix, imgLeft, imgRight)); 

            //* pass frame to tracking and run tracking 
            tracking->addFrameAndTrack(frame); 

            


            frame_counter++; 

        }
        sequenceLeft.release(); 
        sequenceRight.release();
        cv::destroyAllWindows(); 

        visualizer->closeVisualizer(); 


        return 0; 
    }
}