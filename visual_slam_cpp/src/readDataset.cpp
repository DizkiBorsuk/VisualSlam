#include "../include/readDataset.hpp"



mrVSLAM::KITTI_Dataset::KITTI_Dataset(const std::string sequence)
{   
    const std::string sequence_dir = "C:/Users/Maciek/Desktop/dev_workspace/Projects/VisualSlam/KITTY_dataset/sequences//"; 
    const std::string gtp_dir = "C:/Users/Maciek/Desktop/dev_workspace/Projects/VisualSlam/KITTY_dataset/ground_truth_poses//"; 
    const std::string file = ".txt"; 
    const std::string calib_file = "/calib.txt"; 
    const std::string limg = "/image_0/00%04d.png"; 
    const std::string rimg = "/image_1/00%04d.png"; 

    gt_poses_path = gtp_dir + sequence + file; 
    camera_calibration_path = sequence_dir + sequence + calib_file; 
    left_imgs_path = sequence_dir + sequence + limg; 
    right_imgs_path = sequence_dir  + sequence + rimg;
} 

void mrVSLAM::KITTI_Dataset::readCalibData()
{
    std::ifstream calib_file; 
    calib_file.open(camera_calibration_path);
    //calib_file.open(file_path); 

    if(calib_file.fail())
    {
        std::cerr << "Failed to open calib file \n";  
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


void mrVSLAM::KITTI_Dataset::getGTposes()
{
    std::ifstream gt_poses_file; 
    gt_poses_file.open(gt_poses_path.c_str());
    //gt_poses_file.open(file_path); 

    if(gt_poses_file.fail())
    {
        std::cerr << "Failed to open ground truth poses file \n";  
    }

    std::string string_row;
    std::string row_entries;  
    std::vector<double> gt_poses_file_vec; 
    std::vector<double> pose_vec; 
    int number_of_gtposes = 0; 

    while(std::getline(gt_poses_file, string_row))
    {
        std::stringstream row_data_stream(string_row);
        while(std::getline(row_data_stream, row_entries, ' '))
        {
            gt_poses_file_vec.push_back(std::stod(row_entries)); 
        }
        number_of_gtposes++; 
    }

    for(auto i = 0; i < 12*number_of_gtposes; i+=12)
    {
        for(int j = i; j < i+12; j++)
        {
            pose_vec.push_back(gt_poses_file_vec[j]); 
        }
        Eigen::Map<Eigen::Matrix<double,3,4, Eigen::RowMajor>> pose_matrix(pose_vec.data());
        ground_truth_poses.push_back(pose_matrix); 
        pose_vec.clear();
    }
    std::cout << "Number of poses in sequence: " << ground_truth_poses.size() << "\n"; 
}
