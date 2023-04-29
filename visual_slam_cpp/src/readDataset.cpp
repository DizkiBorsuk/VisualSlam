#include "../include/readDataset.hpp"




void mrVSLAM::KITTI_Dataset::readCalibData(std::string file_path)
{
    std::ifstream calib_file; 
    calib_file.open(file_path); 

    if(calib_file.fail())
    {
        std::cerr << "Fail to open calib file \n";  
    }

    std::string string_row;
    std::string row_entries;  
    std::vector<double> calib_data_vec; 


    while(std::getline(calib_file, string_row))
    {
        std::stringstream row_data_stream(string_row);
        while(std::getline(row_data_stream, row_entries, ' '))
        {
            try
            {
                calib_data_vec.push_back(std::stod(row_entries)); 
            }
            catch(...)
            {
                continue;
            }
        }
    }

    Eigen::Map<Eigen::Matrix<double,4,12, Eigen::RowMajor>> calib_data_matrix(calib_data_vec.data());
    P0 = calib_data_matrix.row(0); 
    P0.resize(3,4); 
    P1 = calib_data_matrix.row(0); 
    P1.resize(3,4); 
    P2 = calib_data_matrix.row(0); 
    P2.resize(3,4); 
    P3 = calib_data_matrix.row(0); 
    P3.resize(3,4); 

}   




/*


readDataset::readDataset(seqe)
{
    std::string sequence_dir = "../KITTY_dataset/sequences/" + sequence + "/"; 
    std::string gt_poses_dir = "../KITTY_dataset/ground_truth_poses/" + sequence + ".txt"; 
    std::string left_imgs_path = "C:/Users/Maciek/Desktop/dev_workspace/Projects/VisualSlam/KITTY_dataset/sequences/" + sequence + "/image_0/00%04d.png"; 
    std::string right_imgs_path = "C:/Users/Maciek/Desktop/dev_workspace/Projects/VisualSlam/KITTY_dataset/sequences/" + sequence + "/image_1/00%04d.png"; 
    std::string cam_valib_path = "'../KITTY_dataset/sequences/" + sequence + "/calib.txt";

}
*/