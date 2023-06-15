#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    class stereoSLAM
    {
    private:
    public: 
    int f_counter = 0;  

    stereoSLAM() noexcept {}; 
    stereoSLAM(const Eigen::Matrix<double,3,4> &P_left, const Eigen::Matrix<double,3,4> &P_right); 
    
    }; 





}
