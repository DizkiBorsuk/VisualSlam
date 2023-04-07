#include <iostream>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using std::cout;
using std::endl;

std::string img_path = "C:/Users/Maciek/Desktop/dev_workspace/Projects/VisualSlam/KITTY_dataset/sequences/00/image_0/00%04d.png"; 

int main(int argc, char** argv)
{
    std::cout << "Visual Slam start \n";  

    cv::Mat left_frame, right_frame;  

    cv::VideoCapture sequence; 
    sequence.open(path, cv::CAP_IMAGES);
    
    if (!sequence.isOpened())
    {
      std::cerr << "Failed to open Image Sequence!\n" << endl;
      return 1;
    }
    
    cv::namedWindow("Camera Img", cv::WINDOW_NORMAL); 

    while(true)
    {
        sequence >> left_frame; 
        
        if(left_frame.empty())
        {
            cout << "End of sequance \n"; 
            break;
        }
             
        cv::imshow("Camera Img", left_frame);
        char key = (char)cv::waitKey(66);
        if(key == 'q' || key == 'Q' || key == 27)
            break;
    }
    cv::destroyWindow("Camera Img");
    return 0; 
}