#include "../include/visualize_data.hpp"


void mrVSLAM::plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> gt_poses, 
                        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> poses, int num_of_frames)
{
    std::vector<double> gt_x(num_of_frames), gt_y(num_of_frames),gt_z(num_of_frames), 
                        x(num_of_frames), y(num_of_frames), z(num_of_frames); 

    plt::figure(); 
    //plt::plot(); 

}