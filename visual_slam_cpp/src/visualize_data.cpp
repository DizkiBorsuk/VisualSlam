#include "../include/visualize_data.hpp"

namespace plt = matplotlibcpp; 

inline void mrVSLAM::plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &gt_poses, 
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

inline void mrVSLAM::plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, const int num_of_frames)
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

void mrVSLAM::plotPerformance(std::vector<int> loopTimes)
{
    int sum = std::accumulate(loopTimes.begin(), loopTimes.end(), 0.0); 
    double mean = sum/loopTimes.size(); 

    std::cout << "------- Results --------- \n"; 
    std::cout << "mean fps = " << mean << "\n"; 

    plt::figure(); 
    plt::plot(loopTimes); 
    plt::xlabel(" loops []");
    plt::ylabel("loop times [ms]"); 
    plt::show(); 
}


/*
void mrVSLAM::plotPoses3d(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, const int num_of_frames) 
{

    std::vector<double> gt_x(num_of_frames), gt_y(num_of_frames),gt_z(num_of_frames), 
                        x(num_of_frames), y(num_of_frames), z(num_of_frames); 
    Eigen::Matrix<double,3,4> tmp_pose_matrix; 

    for(int i = 0; i < num_of_frames; i++)
    {
        tmp_pose_matrix = gt_poses[i]; 
        gt_x.push_back(tmp_pose_matrix.coeff(0,3));  
        gt_y.push_back(tmp_pose_matrix.coeff(2,3));
        gt_z.push_back(tmp_pose_matrix.coeff(3,3)); 
    }

    plt::plot3(gt_x,gt_y,gt_z); 
    plt::xlabel("x [m]");
    plt::ylabel("y [m]"); 
    plt::legend();   
    plt::show(); 

}
*/