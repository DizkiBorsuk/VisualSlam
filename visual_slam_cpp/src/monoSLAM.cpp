

namespace mrVSLAM
{
    // void monoSLAM::poseEstimationEpiCons(std::vector<std::vector<cv::Point2f>> &matched_points)
    // {   
    //     std::vector<cv::Point2f> points_frame1, points_frame2; 

    //     std::cout << "mathced points size = " << matched_points.size() << "\n"; 
    //     if(matched_points.size() >= 8)
    //     {
    //         for(int i = 0; i < matched_points.size(); i++)
    //         {
    //             points_frame1.emplace_back(matched_points[i][0]); 
    //             points_frame2.emplace_back(matched_points[i][1]); 
    //         }

    //         F = cv::findFundamentalMat(points_frame1, points_frame2, cv::FM_8POINT); 
    //         std::cout << "F = " << F << "\n"; 
    //         E = cv::findEssentialMat(points_frame1, points_frame2, fy, principialPoint); 
    //         H = cv::findHomography(points_frame1, points_frame2, cv::RANSAC, 3); 
    //         cv::recoverPose(E, points_frame1, points_frame2, R, t, fy, principialPoint);
    //         std::cout << "R = " << R <<"\n t = " << t << "\n";
    //         cv::cv2eigen(R,R_e); 
    //         cv::cv2eigen(t, t_e); 
            
    //         Rt << R_e, t_e; 
            
    //         std::cout << "Rt = " << Rt <<"\n";
    //         poses.emplace_back(Rt); 
    //     }
  
    // }

}

