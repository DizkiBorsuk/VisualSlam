#include "../include/tools.hpp"

namespace plt = matplotlibcpp; 

namespace mrVSLAM
{

    inline void triangulate()
    {
        Eigen::MatrixXd
    }


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


    void plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &gt_poses, 
                        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &poses, const int num_of_frames)
    {
        /*
        Function to visualize estimated poses and ground truth poses on 2D plot 
        */
        std::vector<double> gt_x(num_of_frames), gt_y(num_of_frames), 
                            x(num_of_frames), y(num_of_frames); 
        Eigen::Matrix<double,3,4> tmp_pose_matrix; 

        for(int i = 0; i < num_of_frames; i++)
        {
            tmp_pose_matrix = gt_poses[i]; 
            gt_x.push_back(tmp_pose_matrix.coeff(0,3));  
            gt_y.push_back(tmp_pose_matrix.coeff(2,3));
            tmp_pose_matrix = poses[i]; 
            x.push_back(tmp_pose_matrix.coeff(0,3));  
            y.push_back(tmp_pose_matrix.coeff(2,3));

        }
        plt::figure(); 
        plt::plot(gt_x, gt_y); 
        plt::plot(x, y, "r--");
        plt::xlabel("x [m]");
        plt::ylabel("y [m]"); 
        plt::legend();   
        plt::show(); 

    }

    void plotPoses(std::vector<cv::Matx44d>& poses, std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, const int num_of_frames)
    {
        std::vector<double> gt_x(num_of_frames), gt_y(num_of_frames), 
                            x(num_of_frames), y(num_of_frames); 

        Eigen::Matrix<double,3,4> tmp_pose_matrix;
        cv::Matx44d tmp_pose;  

        for(int i = 0; i < num_of_frames; i++)
        {
            tmp_pose_matrix = gt_poses[i]; 
            gt_x.push_back(tmp_pose_matrix.coeff(0,3));  
            gt_y.push_back(tmp_pose_matrix.coeff(2,3));
            tmp_pose = poses[i]; 
            x.push_back(tmp_pose(0,3));  
            y.push_back(tmp_pose(2,3));

        }
        plt::figure(); 
        plt::plot(gt_x, gt_y); 
        plt::plot(x, y, "r--");
        plt::xlabel("x [m]");
        plt::ylabel("y [m]"); 
        plt::legend();   
        plt::show();
    }

    void plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, const int num_of_frames)
    {
        std::vector<double> gt_x(num_of_frames), gt_y(num_of_frames),gt_z(num_of_frames); 
        Eigen::Matrix<double,3,4> tmp_pose_matrix; 

        for(int i = 0; i < num_of_frames; i++)
        {
            tmp_pose_matrix = gt_poses[i]; 
            gt_x.push_back(tmp_pose_matrix.coeff(0,3));  
            gt_y.push_back(tmp_pose_matrix.coeff(2,3));
        }
    
        plt::figure(); 
        plt::plot(gt_x, gt_y); 
        plt::xlabel("x [m]");
        plt::ylabel("y [m]"); 
        plt::show(); 
    }

    void plotPerformance(std::vector<int> loopTimes)
    {
        int sum = std::accumulate(loopTimes.begin(), loopTimes.end(), 0.0); 
        double mean = sum/loopTimes.size(); 

        std::cout << "------- Results --------- \n"; 
        std::cout << "mean fps = " << mean << "\n"; 
        std::cout << "min value = " << *std::min_element(loopTimes.begin(), loopTimes.end()) << "\n"; 

        plt::figure(); 
        plt::plot(loopTimes); 
        plt::xlabel(" loops []");
        plt::ylabel("loop times [ms]"); 
        plt::show(); 
    }
}
