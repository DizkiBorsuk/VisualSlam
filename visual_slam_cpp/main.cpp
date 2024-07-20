#include "mrVSLAM/common_includes.hpp"
#include "mrVSLAM/slam.hpp"


int main(int argc, char* argv[])
{
    std::string kitti_path = "/home/maciek/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06";
    DatasetVersion dataset_to_run = DatasetVersion::KITTI;
    SLAM_TYPE slam_trackin_type = SLAM_TYPE::STEREO;
    DetectorType detector = DetectorType::GFTT;
    bool use_loop_closer = false;
    unsigned int n_points = 150;
    unsigned int n_min_tracking_points = 80;
    bool plot_results = true;
    float img_size = 1.f;

    fmt::print("Hello mrVSLAM \n");

    //input argumes handling
    if(argc>1)
    {
        fmt::print(fg(fmt::color::green), "using terminal input arguments \n");
        try
        {
            kitti_path = argv[1];
            dataset_to_run = static_cast<DatasetVersion>(std::stoi(argv[2]));
            slam_trackin_type = static_cast<SLAM_TYPE>(std::stoi(argv[3]));
            detector = static_cast<DetectorType>(std::stoi(argv[4]));
            use_loop_closer = std::stoi(argv[5]);
            n_points = std::stoi(argv[6]);
            n_min_tracking_points = std::stoi(argv[7]);
            plot_results = std::stoi(argv[8]);

            fmt::print(fg(fmt::color::green), "kitti path = {}, usle_loop_closer = {}, n_points = {} \n", kitti_path, use_loop_closer, n_points);
        }
        catch(const std::exception& e)
        {
            fmt::print(fg(fmt::color::red), "Error in reading input parameters {}, using default", e.what());
        }

    } else { fmt::print(fg(fmt::color::red),"no input arguments, using default inputs \n"); }

    //creat SLAM class instance
    auto slam = std::make_shared<mrVSLAM::SLAM>(kitti_path, dataset_to_run, slam_trackin_type, use_loop_closer);

    // run SLAM
    slam->setSlamParameters(detector, n_points, n_min_tracking_points, img_size, plot_results);
    slam->initSLAM();
    slam->runSLAM();
    slam->outputSlamResult(plot_results);

    return 0;
}
