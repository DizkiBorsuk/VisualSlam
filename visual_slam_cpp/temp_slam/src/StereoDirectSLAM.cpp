#include "../include/StereoDirectSLAM.hpp"



namespace mrVSLAM
{
    StereoDirectSLAM::StereoDirectSLAM()
    {
        ptr_to_map = std::shared_ptr<Map>(new Map); 
        ptr_to_visualization = std::shared_ptr<Visualization>(new Visualization); 
    }


    StereoDirectSLAM::Run()
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

        while(true)
        {
            sequence.read(img);
            if(img.empty()) {
                std::cout << "End of sequance \n"; 
                break;
            }

            loopStart = cv::getTickCount(); 
            auto begin = std::chrono::high_resolution_clock::now();

            

        }

        ptr_to_visualizer->closeVisualizer(); 

    }
}