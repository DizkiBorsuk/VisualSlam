
#include "system.hpp"


namespace mrVSLAM
{
    enum MatcherType {BruteForce, Flann}; 
    enum ExtractorType {orb, orb_fast, orb_harris, orb_gptt, sift, akaze}; 

    class featureVO
    {
    private: 
        cv::Mat H, F, K, E, t_h = cv::Mat(4,1,CV_8UC1), t = cv::Mat(3,1,CV_8UC1); 

        Eigen::Matrix3d R_e; 
        Eigen::Vector3d t_e; 



        void poseEstimationEpiCons(std::vector<std::vector<cv::Point2f>> &matched_points); 

    public:
        int f_counter = 0; 
        std::array<double,6> x_t; // object state x_t = [x,y,z, phi,theta,gamma]
        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> poses;
        Eigen::Matrix<double, 3,4, Eigen::RowMajor> Rt; 




    }; 


}


