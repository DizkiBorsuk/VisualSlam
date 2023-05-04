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
//my headers
#include "../include/readDataset.hpp"
#include "../include/visualize_data.hpp"



int counter = 0; 

int main(int argc, char** argv)
{
    std::cout << "Visual Slam start \n";  
    cv::cuda::printCudaDeviceInfo(0); 

    ///// ------ Read Calibration data and Ground Truth Poses ---- //////
    mrVSLAM::KITTI_Dataset kitti("07"); 
    //kitti.readCalibData(); 
    kitti.showPmatricies(); 

    kitti.getGTposes(); 
    std::cout << kitti.ground_truth_poses[0] <<  "\n"; 
    //std::setprecision(1) <<std::fixed<<
         

    std::cout << "\n" << "-------------"<<"\n"; 


    ////// ---------- /////////////
    cv::Mat left_frame, right_frame;  
    cv::Mat stereo; 
    cv::cuda::GpuMat l_frame, r_frame; 

    cv::namedWindow("Camera Img", cv::WINDOW_NORMAL); 

    cv::VideoCapture left_sequence; 
    cv::VideoCapture right_sequence; 
    left_sequence.open(kitti.left_imgs_path, cv::CAP_IMAGES);
    right_sequence.open(kitti.right_imgs_path, cv::CAP_IMAGES); 

    if (!left_sequence.isOpened())
    {
      std::cerr << "Failed to open Image Sequence!\n"; 
      return 1;
    }

    if (!right_sequence.isOpened())
    {
      std::cerr << "Failed to open right Image Sequence!\n"; 
      return 1;
    }

    
    while(true)
    {
        auto start = cv::getTickCount(); 

        left_sequence.read(left_frame);
        right_sequence.read(right_frame); 

        if(left_frame.empty())
        {
            std::cout << "End of sequance \n"; 
            break;
        }
        
        //l_frame.upload(left_frame); 

        //////// ----- Algorithm body ------ /////////





        //////// ----- Algorithm End ----- //////////

        //l_frame.upload(left_frame); 
        
        cv::hconcat(left_frame, right_frame, stereo); 

        auto end = cv::getTickCount(); 
        auto framesPerSecond = 1/((end - start)/cv::getTickFrequency()); 

        cv::putText(stereo, "fps :" + std::to_string(int(framesPerSecond)), cv::Point(30,50), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0,0,0),2);
        cv::imshow("Camera Img", stereo);
        std::cout << "Frame num: " << counter++ << "\n"; 

        char key = (char)cv::waitKey(66); 
        if(key == 'q' || key == 27)
            break;
    }
    cv::destroyAllWindows(); 

    /* Results */

    mrVSLAM::plotPoses(kitti.ground_truth_poses, counter); 


    return 0; 
}