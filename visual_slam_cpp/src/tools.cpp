#include "..\include\tools.hpp"

namespace mrVSLAM
{
    void getTransformationMatrix(const cv::Matx33d &R,const cv::Matx31d &t, cv::Matx44d &outT)
    {
        cv::Mat Rt, last_row; 
        cv::hconcat(R, t, Rt); 

        last_row.at<double>(0,0) = 0; 
        last_row.at<double>(0,1) = 0;
        last_row.at<double>(0,2) = 0;
        last_row.at<double>(0,3) = 1;
        cv::vconcat(Rt, last_row, outT); 
    }
}