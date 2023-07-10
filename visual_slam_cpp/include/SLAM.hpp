#include "system.hpp"
#include "camera.hpp"
#include "readDataset.hpp"
#include "visualize_data.hpp"

namespace mrVSLAM
{
    class SLAM
    {
    private: 
    // basic 
        int frame_counter = 0; 
        int fps = 0; 
        
        KITTI_Dataset dataset; 
    // mono 
        //std::unique_ptr<Camera> camera; 
        Camera camera; 
    //stereo 
        //std::unique_ptr<Camera> right_camera; 
        Camera right_camera; 
        double baseline = 0; 


    public: 
        enum SlamType {featureMono, featureStereo, direct}; 
        
        ~SLAM(); 
        SLAM(SlamType, std::string sequence_number); 
        
        // delete move and copy constructrs/operators
        SLAM(const SLAM&) = delete;  
        SLAM(SLAM&&) = delete;
        SLAM& operator=(const SLAM&) = delete; 
        SLAM& operator=(const SLAM&&) = delete; 

        //void initSLAM(); 
        int runSLAM(); 
        int runGpuSLAM();  

        void showResult(); 
    }; 

}



