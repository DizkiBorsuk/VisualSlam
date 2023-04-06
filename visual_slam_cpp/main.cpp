#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using std::cout;
using std::endl;


int main()
{
    std::cout << "Visual Slam start \n";  
    cv::RNG rng(12345);
    cv::Mat img, img2; 
    img = cv::imread("C:/Users/Maciek/Desktop/dev_workspace/Projects/VisualSlam/KITTY_dataset/sequences/00/image_0/000000.png"); 
    cv::cvtColor(img,img2,cv::COLOR_BGR2GRAY); 
    std::vector<cv::Point2f> corners; 
    cv::goodFeaturesToTrack( img2, corners, 1000, 0.01, 10, cv::Mat(), 3,3,false, 0.04);

    for(auto i : corners)
    {
        cv::circle(img,i,4,cv::Scalar(rng.uniform(0,255), rng.uniform(0, 256), rng.uniform(0, 256)),3); 

    }

    cv::imshow("img",img);
    cv::waitKey(0);
    cv::destroyWindow("img");


    return 0; 
}