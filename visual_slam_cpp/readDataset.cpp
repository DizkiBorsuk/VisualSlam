#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <string>

using namespace Eigen;

template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}


class readDataset
{
private: 
public:
    std::string sequence_dir; 
    std::string gt_poses_dir; 
    std::string left_imgs_path; 
    std::string right_imgs_path; 
    std::string cam_valib_path;
    readDataset(std::string sequence = "00");
    ~readDataset();

     
};

readDataset::readDataset(std::string sequence = "00")
{
    std::string sequence_dir = "../KITTY_dataset/sequences/" + sequence + "/"; 
    std::string gt_poses_dir = "../KITTY_dataset/ground_truth_poses/" + sequence + ".txt"; 
    std::string left_imgs_path = "C:/Users/Maciek/Desktop/dev_workspace/Projects/VisualSlam/KITTY_dataset/sequences/" + sequence + "/image_0/00%04d.png"; 
    std::string right_imgs_path = "C:/Users/Maciek/Desktop/dev_workspace/Projects/VisualSlam/KITTY_dataset/sequences/" + sequence + "/image_1/00%04d.png"; 
    std::string cam_valib_path = "'../KITTY_dataset/sequences/" + sequence + "/calib.txt";

}

readDataset::~readDataset()
{
}

