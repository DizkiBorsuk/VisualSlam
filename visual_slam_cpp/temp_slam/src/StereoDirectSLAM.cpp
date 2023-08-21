#include "../include/StereoDirectSLAM.hpp"



namespace mrVSLAM
{
    StereoDirectSLAM::StereoDirectSLAM()
    {
        ptr_to_map = std::shared_ptr<Map>(new Map); 
        ptr_to_visualization = std::shared_ptr<Visualization>(new Visualization); 
    }



}