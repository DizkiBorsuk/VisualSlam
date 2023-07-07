
#include "../include/SLAM.hpp"

namespace mrVSLAM
{
   SLAM::~SLAM() = default;  
   SLAM::SLAM(SlamType typeIn, std::string sequence_number) 
   {
        dataset.chooseSequence(sequence_number); 
        dataset.readCalibData(); 
        dataset.showPmatricies(); 
        dataset.getGTposes(); 
        std::cout << std::setprecision(1) << std::fixed << dataset.ground_truth_poses[0] <<  "\n"; 
        std::cout << "\n" << "-------------"<<"\n"; 


        switch (typeIn)
        {
        case featureMono:
            camera.setCamera(dataset.P0); 
            std::cout << "fx =" <<camera.fx << "\n"; 
            break;
        case featureStereo: 
            camera.setCamera(dataset.P0);
            right_camera.setCamera(dataset.P1);  
            baseline = getStereoBaseline(camera.t_mat, right_camera.t_mat); 
            std::cout << "baseline = " << baseline << "\n"; 
            break; 
        case direct: 
            break; 
        default:
            break;
        }
   }

   int runSLAM()
   {

        return 0; 
   }
}