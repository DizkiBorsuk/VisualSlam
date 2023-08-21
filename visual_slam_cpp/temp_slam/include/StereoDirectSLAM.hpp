#pragma once 
#include "common_includes.hpp"

namespace mrVSLAM
{
    class StereoDirectSLAM
    {
    public: 
        StereoDirectSLAM(); 
        void Run(); 
    private: 
        bool initialization_succes = false; 
        std::shared_ptr<Map> ptr_to_map = nullptr; 
        std::shared_ptr<Visuzalization> ptr_to_visualization = nullptr; 
        //std::shared_ptr<> ptr_ = nullptr; 


    };
}

 