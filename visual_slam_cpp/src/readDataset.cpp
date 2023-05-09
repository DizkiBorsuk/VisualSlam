#include "../include/readDataset.hpp"



mrVSLAM::KITTI_Dataset::KITTI_Dataset(const std::string& sequence)
{   
    /*
        Inputs: sequence - number of sequence from kitti dataset (07, 06, 00)
    */
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
    /*
    Calib.txt contains camera calibration data -> Projection matricies (3x4), projection matrix contains intrinsic (focal lengths, camera center) and extrinsic parameters
    Kitti dataset stores projection matricies in flat shape, that is as a array with 12 elemets, each row is diffrent matrix
    */
    std::ifstream calib_file; 
    calib_file.open(camera_calibration_path); //open calibration file 

    if(calib_file.fail())
    {
        std::cerr << "Failed to open calib file \n";  
    }

    std::string string_row;
    std::string row_entries;  
    std::vector<double> calib_data_vec; 

    while(std::getline(calib_file, string_row))
    {
        std::stringstream row_data_stream(string_row); // get from file a whole line as one string
        while(std::getline(row_data_stream, row_entries, ' ')) //separate strings from line based on spaces 
        {
            try
            {
                calib_data_vec.push_back(std::stod(row_entries)); // convert data to doubles
            }
            catch(...)
            {
                continue; //first in every line is Pn: so conversion to double will fail, I know it's a bit junky way to solve it 
            }
        }
    }

    Eigen::Map<Eigen::Matrix<double,4,12, Eigen::RowMajor>> calib_data_matrix(calib_data_vec.data()); // map data to 4x12 eigen matrix (4 projection matricies, every with 12 elements (3x4))
    // Get rows from calib_data_matrix assign them to seprate projection matricies and resize them to proper size 
    P0 = calib_data_matrix.row(0);  
    P0.resize(3,4); 
    P1 = calib_data_matrix.row(1); 
    P1.resize(3,4); 
    P2 = calib_data_matrix.row(2); 
    P2.resize(3,4); 
    P3 = calib_data_matrix.row(3); 
    P3.resize(3,4); 
}   


void mrVSLAM::KITTI_Dataset::getGTposes()
{
    /*
    files with ground truth poses contain poses of car thru sequence, poses are represented as 3x4 Transformation matrix
    Kitti dataset stores translation matricies in flat shape, that is as a array with 12 elemets, each row is diffrent matrix
    Output of function is vector of translation matricies
    */
    std::ifstream gt_poses_file; 
    gt_poses_file.open(gt_poses_path.c_str()); //open gtround truth poses file
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

    while(std::getline(gt_poses_file, string_row)) //
    {
        std::stringstream row_data_stream(string_row); // get from file a whole line as one string
        while(std::getline(row_data_stream, row_entries, ' ')) //separate strings from line based on spaces
        {
            gt_poses_file_vec.push_back(std::stod(row_entries)); //translate obtained strings to double and put them in one vector
        }
        number_of_gtposes++; // count number of poses (rows in file)
    }


    // from vector with all entries from file get 12 element vector and map it to eigen matrix (Transformation matrix)
    // append matrix to ground_truth_poses vector to create sequence of matricies
    for(auto i = 0; i < 12*number_of_gtposes; i+=12) // loop thru all elements, jumping every row in file 
    {
        for(int j = i; j < i+12; j++) //loop thru every element in row 
        {
            pose_vec.push_back(gt_poses_file_vec[j]); 
        }
        Eigen::Map<Eigen::Matrix<double,3,4, Eigen::RowMajor>> pose_matrix(pose_vec.data());
        ground_truth_poses.push_back(pose_matrix); 
        pose_vec.clear(); //clear vector 
    }
    std::cout << "Number of poses in sequence: " << ground_truth_poses.size() << "\n"; 
}


void mrVSLAM::KITTI_Dataset::showPmatricies()
{
    std::cout << "---------------- \n"; 
    std::cout << "Left grayscale camera projection matrix = \n" << P0 << "\n"; 
    std::cout << "\n"; 
    std::cout << "Right grayscale camera projection matrix = \n" << P1 << "\n"; 
    std::cout << "---------------- \n";
}