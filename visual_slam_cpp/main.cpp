#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudafeatures2d.hpp>


std::string left_img_path = "C:/Users/Maciek/Desktop/dev_workspace/Projects/VisualSlam/KITTY_dataset/sequences/00/image_0/00%04d.png"; // path to img sequence 
int counter = 0; 

int main(int argc, char** argv)
{
    std::cout << "Visual Slam start \n";  
    cv::cuda::printCudaDeviceInfo(0); 

    ///// ------ Read Calibration data and Ground Truth Poses ---- //////




    ////// ---------- /////////////
    cv::Mat left_frame, right_frame;  
    cv::cuda::GpuMat l_frame, r_frame; 

    cv::namedWindow("Camera Img", cv::WINDOW_AUTOSIZE); 

    cv::VideoCapture left_sequence; 
    left_sequence.open(left_img_path, cv::CAP_IMAGES);
    
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
        
        l_frame.upload(left_frame); 

        //////// ----- Algorithm body ------ /////////





        //////// ----- Algorithm End ----- //////////

        l_frame.upload(left_frame); 

        auto end = cv::getTickCount(); 
        auto framesPerSecond = 1/((end - start)/cv::getTickFrequency()); 

        cv::putText(left_frame, "fps :" + std::to_string(int(framesPerSecond)), cv::Point(30,50), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0,0,0),2);
        cv::imshow("Camera Img", left_frame);
        std::cout << "Frame num: " << counter++ << "\n"; 

        char key = (char)cv::waitKey(66); 
        if(key == 'q' || key == 27)
            break;
    }
    cv::destroyAllWindows(); 
    return 0; 
}