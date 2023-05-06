#include "../include/visualize_data.hpp"


void mrVSLAM::plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, 
                        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& poses, const int num_of_frames)
{
    /*
    Function to visualize estimated poses and  ground truth poses on 2D plot 
    */
    std::vector<double> gt_x(num_of_frames), gt_y(num_of_frames),gt_z(num_of_frames), 
                        x(num_of_frames), y(num_of_frames), z(num_of_frames); 
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
    plt::xlabel("x [m]");
    plt::ylabel("y [m]"); 
    plt::plot(gt_x, gt_y); 
    plt::plot(x, y, "r--");
    plt::legend();   
    plt::show(); 

}

void mrVSLAM::plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, const int num_of_frames)
{
    std::vector<double> gt_x(num_of_frames), gt_y(num_of_frames),gt_z(num_of_frames), 
                        x(num_of_frames), y(num_of_frames), z(num_of_frames); 
    Eigen::Matrix<double,3,4> tmp_pose_matrix; 

    for(int i = 0; i < num_of_frames; i++)
    {
        tmp_pose_matrix = gt_poses[i]; 
        gt_x.push_back(tmp_pose_matrix.coeff(0,3));  
        gt_y.push_back(tmp_pose_matrix.coeff(2,3));
    }

    plt::figure();  
    plt::xlabel("x [m]");
    plt::ylabel("y [m]"); 
    plt::plot(gt_x, gt_y); 
    plt::show(); 
}