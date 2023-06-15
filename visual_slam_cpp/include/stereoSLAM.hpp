#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    class StereoSLAM
    {
    private:
    public: 
    int f_counter = 0;  

    StereoSLAM() noexcept {}; 
    StereoSLAM(const Eigen::Matrix<double,3,4> &P_left, const Eigen::Matrix<double,3,4> &P_right); 
    int executeStereoSLAM(const std::string& left_imgs_path, const std::string& right_imgs_path); 
    }; 





}
