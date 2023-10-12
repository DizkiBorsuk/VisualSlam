#include <iostream>
#include <array>
#include <vector>
#include <algorithm>
#include "eigen3/Eigen/Dense"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include "sophus/se3.hpp"



bool triangulate(const std::array<Eigen::Vector3d,2> &points,const Eigen::Matrix4d &T1, const Eigen::Matrix4d &T2, Eigen::Vector3d &out_point_pos)
{
   //? https://www.cs.cmu.edu/~16385/s17/Slides/11.4_Triangulation.pdf
    Eigen::MatrixXd A(4,4); //! it can't be static size  matrix ?thou i think it worked on windows

    A.row(0) = points[0][0]*T1.row(2) - T1.row(0); 
    A.row(1) = points[0][1]*T1.row(2) - T1.row(1); 
    A.row(2) = points[1][0]*T2.row(2) - T2.row(0); 
    A.row(3) = points[1][1]*T2.row(2) - T2.row(1); 

    //* svd decomposition of A matrix 
    //auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV); //? https://eigen.tuxfamily.org/dox/group__SVD__Module.html
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
    
    Eigen::Vector4d point_homo = svd.matrixV().col(3);
    out_point_pos = point_homo.head(3) / point_homo(3); // convert to cartesian from homogenous 
    if(out_point_pos(2) <= 0)
    {
        return false; 
    } 
    
    return true;
}


auto detector = cv::GFTTDetector::create(500, 0.01, 20, 3, false, 0.04); 
//auto detector = cv::FastFeatureDetector::create(40, true, cv::FastFeatureDetector::TYPE_9_16); 


int main()
{

    cv::Mat imgleft = cv::imread("../KITTY_dataset/sequences/07/image_0/000000.png"); 
    cv::Mat imgright = cv::imread("../KITTY_dataset/sequences/07/image_1/000000.png"); 


    std::vector<cv::KeyPoint> keypoints; 
    detector->detect(imgleft, keypoints, cv::noArray()); 
    // detector->detect(imgright, keypoints_right, cv::noArray()); 

    std::vector<cv::Point2f> keypoints_left, keypoints_right; 

        for(auto &point :keypoints)
        {

                keypoints_left.emplace_back(point.pt); 
                keypoints_right.emplace_back(point.pt);
        }

        std::vector<uchar> status; // output status vector (of unsigned chars); each element of the vector is set to 1 if the flow for the corresponding features has been found, otherwise, it is set to 0.
        cv::Mat err; 
        cv::calcOpticalFlowPyrLK(imgleft, imgright, keypoints_left, keypoints_right, status, err, cv::Size(11,11), 3, 
                                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01), cv::OPTFLOW_USE_INITIAL_FLOW); 
        unsigned int foundCorrespondences = 0; 
        for(unsigned int i = 0; i < status.size(); i++)
        {
            if(status[i] == 1)
            {
                cv::KeyPoint keypoint_in_right(keypoints_right[i], 7); // keypoint pos and size 
                std::array<>
            }
        }



    return 0; 
}
