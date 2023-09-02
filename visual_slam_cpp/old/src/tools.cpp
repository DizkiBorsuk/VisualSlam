#include "..\include\tools.hpp"

namespace mrVSLAM
{
    void getTransformationMatrix(const cv::Matx33d &R,const cv::Matx31d &t, cv::Matx44d &outT)
    {
        // function that changes 3x3 rotation matrix and 3x1 ranslation vector to 4x4 homogenouse tranformation/pose matrix 
        cv::Matx34d Rt;  
        cv::Matx14d last_row; 
        cv::hconcat(R, t, Rt); 

        last_row(0,0) = 0; 
        last_row(0,1) = 0;
        last_row(0,2) = 0;
        last_row(0,3) = 1;

        cv::vconcat(Rt, last_row, outT); 
    }
}