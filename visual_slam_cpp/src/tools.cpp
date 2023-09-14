#include "../include/tools.hpp"

namespace plt = matplotlibcpp; 

namespace mrVSLAM
{

    inline bool triangulate(const std::vector<Eigen::Matrix<double,2,1>> &points,const Eigen::Matrix<double,3,4> &P1, const Eigen::Matrix<double,3,4> &P2, std::array<float,3> &point_pos)
    {
        /* triangulate one point, 
        In: 
        points - corresponding points expressed in camera coardinate system //https://www.youtube.com/watch?v=UZlRhEUWSas&list=PLgnQpQtFTOGQEXN2Qo571uvwIGNGAM8uf&index=11&t=371s
        P - projection matrix matrix 
        K * [R|t] = [fx 0 cx; 0 fy cy; 0 0 1] * [R11 R12 R13; ...] [x; y; z ] 
        Out: 
        position of point in 3d 
        */
       //? https://www.cs.cmu.edu/~16385/s17/Slides/11.4_Triangulation.pdf

        Eigen::Matrix4d A; // 
        /*
        A = [y1p2^T - p1^T; p0^T - x1p2^T; y2p2^T - p1^T; p0^T - x2p2^T]
        */
    
        A.block<1,4>(0, 0) = points[0][0]*P1.row(2) - P1.row(0); 
        A.block<1,4>(1, 0) = points[0][1]*P1.row(2) - P1.row(1); 
        A.block<1,4>(2, 0) = points[1][0]*P2.row(2) - P2.row(0); 
        A.block<1,4>(3, 0) = points[1][1]*P2.row(2) - P2.row(1); 
    
        //* svd decomposition of A matrix 
        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV); //? https://eigen.tuxfamily.org/dox/group__SVD__Module.html
        point_pos = (svd.matrixV().col(3) / svd.matrixV()(3,3)).head<3>(); //! change point pos to Eigen::Vector3d

        if(svd.singularValues()[3] / svd.singularValues()[2] > 1e-2)
            return false;
        else 
            return true;
    }

    inline bool triangulate(const std::vector<Eigen::Matrix<double,2,1>> &points,const Eigen::Matrix<double,3,4> &P, std::array<float,3> &point_pos)
    {
        Eigen::Matrix4d A; // 
    
        A.block<1,4>(0, 0) = points[0][0]*P.row(2) - P.row(0); 
        A.block<1,4>(1, 0) = points[0][1]*P.row(2) - P.row(1); 
        A.block<1,4>(2, 0) = points[1][0]*P.row(2) - P.row(0); 
        A.block<1,4>(3, 0) = points[1][1]*P.row(2) - P.row(1); 
    
        //* svd decomposition of A matrix 
        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV); //? https://eigen.tuxfamily.org/dox/group__SVD__Module.html
        point_pos = (svd.matrixV().col(3) / svd.matrixV()(3,3)).head<3>(); //! change point pos to Eigen::Vector3d

        if(svd.singularValues()[3] / svd.singularValues()[2] > 1e-2)
            return false;
        else 
            return true;
    }


    inline void getTransformationMatrix(const cv::Matx33d &R,const cv::Matx31d &t, cv::Matx44d &outT)
    {
        // function that changes 3x3 rotation matrix and 3x1 ranslation vector to 4x4 homogenouse tranformation/pose matrix 
        cv::Matx34d Rt;  
        cv::Matx14d last_row; 
        cv::hconcat(R, t, Rt); 

        last_row(0,0) = 0; 
        last_row(0,1) = 0;
        last_row(0,2) = 0;
        last_row(0,3) = 1;

        cv::vconcat(Rt, last_row, outT); 
    }


    void plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &gt_poses, 
                        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &poses, const int num_of_frames)
    {
        /*
        Function to visualize estimated poses and ground truth poses on 2D plot 
        */
        std::vector<double> gt_x(num_of_frames), gt_y(num_of_frames), 
                            x(num_of_frames), y(num_of_frames); 
        Eigen::Matrix<double,3,4> tmp_pose_matrix; 

        for(int i = 0; i < num_of_frames; i++)
        {
            tmp_pose_matrix = gt_poses[i]; 
            gt_x.push_back(tmp_pose_matrix.coeff(0,3));  
            gt_y.push_back(tmp_pose_matrix.coeff(2,3));
            tmp_pose_matrix = poses[i]; 
            x.push_back(tmp_pose_matrix.coeff(0,3));  
            y.push_back(tmp_pose_matrix.coeff(2,3));

        }
        plt::figure(); 
        plt::plot(gt_x, gt_y); 
        plt::plot(x, y, "r--");
        plt::xlabel("x [m]");
        plt::ylabel("y [m]"); 
        plt::legend();   
        plt::show(); 

    }

    void plotPoses(std::vector<cv::Matx44d>& poses, std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, const int num_of_frames)
    {
        std::vector<double> gt_x(num_of_frames), gt_y(num_of_frames), 
                            x(num_of_frames), y(num_of_frames); 

        Eigen::Matrix<double,3,4> tmp_pose_matrix;
        cv::Matx44d tmp_pose;  

        for(int i = 0; i < num_of_frames; i++)
        {
            tmp_pose_matrix = gt_poses[i]; 
            gt_x.push_back(tmp_pose_matrix.coeff(0,3));  
            gt_y.push_back(tmp_pose_matrix.coeff(2,3));
            tmp_pose = poses[i]; 
            x.push_back(tmp_pose(0,3));  
            y.push_back(tmp_pose(2,3));

        }
        plt::figure(); 
        plt::plot(gt_x, gt_y); 
        plt::plot(x, y, "r--");
        plt::xlabel("x [m]");
        plt::ylabel("y [m]"); 
        plt::legend();   
        plt::show();
    }

    void plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, const int num_of_frames)
    {
        std::vector<double> gt_x(num_of_frames), gt_y(num_of_frames),gt_z(num_of_frames); 
        Eigen::Matrix<double,3,4> tmp_pose_matrix; 

        for(int i = 0; i < num_of_frames; i++)
        {
            tmp_pose_matrix = gt_poses[i]; 
            gt_x.push_back(tmp_pose_matrix.coeff(0,3));  
            gt_y.push_back(tmp_pose_matrix.coeff(2,3));
        }
    
        plt::figure(); 
        plt::plot(gt_x, gt_y); 
        plt::xlabel("x [m]");
        plt::ylabel("y [m]"); 
        plt::show(); 
    }

    void plotPerformance(std::vector<int> loopTimes)
    {
        int sum = std::accumulate(loopTimes.begin(), loopTimes.end(), 0.0); 
        double mean = sum/loopTimes.size(); 

        std::cout << "------- Results --------- \n"; 
        std::cout << "mean fps = " << mean << "\n"; 
        std::cout << "min value = " << *std::min_element(loopTimes.begin(), loopTimes.end()) << "\n"; 

        plt::figure(); 
        plt::plot(loopTimes); 
        plt::xlabel(" loops []");
        plt::ylabel("loop times [ms]"); 
        plt::show(); 
    }
}
