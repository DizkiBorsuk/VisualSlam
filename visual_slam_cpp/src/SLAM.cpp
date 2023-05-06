#include "../include/SLAM.hpp"

int mrVSLAM::SLAM::executeMonoSLAM(std::string& imgs_path)
{
    cv::Mat left_frame; 
    cv::cuda::GpuMat gpu_frame; 

    cv::namedWindow("Camera Img", cv::WINDOW_AUTOSIZE); 

    cv::VideoCapture left_sequence; 
    left_sequence.open(imgs_path, cv::CAP_IMAGES);

    if (!left_sequence.isOpened())
    {
      std::cerr << "Failed to open Image Sequence!\n"; 
      return 1;
    }

    while(true)
    {
        auto start = cv::getTickCount(); 

        left_sequence.read(left_frame);

        if(left_frame.empty())
        {
            std::cout << "End of sequance \n"; 
            break;
        }
        
        //l_frame.upload(left_frame); 

        //////// ----- Algorithm body ------ /////////





        //////// ----- Algorithm End ----- //////////

        //l_frame.upload(left_frame); 
        
        //cv::hconcat(left_frame, right_frame, stereo); 

        auto end = cv::getTickCount(); 
        auto framesPerSecond = 1/((end - start)/cv::getTickFrequency()); 

        cv::putText(left_frame, "fps :" + std::to_string(int(framesPerSecond)), cv::Point(30,50), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0,0,0),2);
        cv::imshow("Camera Img", left_frame);
        std::cout << "Frame num: " << f_counter++ << "\n"; 

        char key = (char)cv::waitKey(66); 
        if(key == 'q' || key == 27)
            break;
    }
    cv::destroyAllWindows(); 

    return 0; 
}

