#include "..\include\tools.hpp"

namespace mrVSLAM
{
    Eigen::Matrix3d findFundamentalMatrix(const std::vector<std::vector<cv::Point2f>> &matched_points)
    {
        Eigen::Matrix3d fundamentalMatrix; 
        std::vector<cv::Point2f> points_frame1, points_frame2; 

        for(auto points: matched_points)
        {
            points_frame1.emplace_back(points[0]); 
            points_frame1.emplace_back(points[1]); 
        }



        return fundamentalMatrix; 
    }
    Eigen::Matrix3d findEssentialMatrix(const Eigen::Matrix3d &intrinsicMatrix, const Eigen::Matrix3d &fundamentalMatrix)
    {
        Eigen::Matrix3d essentialMatrix; 


        return essentialMatrix; 
    }





}