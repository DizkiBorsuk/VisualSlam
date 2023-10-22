#include "myslam/tools.hpp"
#include <matplot/matplot.h>

using namespace matplot;

namespace myslam
{

    void plotPoses(std::vector<Eigen::Matrix<double, 3,4>> &poses, 
                   std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &gt_poses)
    {
        std::vector<double> gt_x, gt_y, x, y; 
 
        for(std::size_t i = 0; i < poses.size(); i++)
        {
            gt_x.push_back(gt_poses.at(i).coeff(0,3));  
            gt_y.push_back(gt_poses.at(i).coeff(2,3));

            x.push_back(poses.at(i).coeff(0,3));  
            y.push_back(poses.at(i).coeff(2,3));

        }

        plot(gt_x, gt_y); 
        hold(on); 
        plot(x, y, "r--");
        xlabel("x [m]");
        ylabel("y [m]");  
        show();
    }

    void plotPoses(std::vector<Eigen::Matrix<double, 3,4>> &poses)
    {
        std::vector<double> x, y; 
        std::cout << "trajcetory size " << poses.size() << "\n"; 
 
        for(std::size_t i = 0; i < poses.size(); i++)
        {
            x.push_back(poses.at(i).coeff(0,3));  
            y.push_back(poses.at(i).coeff(1,3));
        }

        plot(x, y, "r--");
        xlabel("x [m]");
        ylabel("y [m]"); 
        show();
    }
    void plotPerformance(std::vector<int> loopTimes)
    {

        int sum = std::accumulate(loopTimes.begin(), loopTimes.end(), 0.0); 
        double mean = sum/loopTimes.size(); 

        std::cout << "mean fps = " << mean << "\n"; 
        std::cout << "min value = " << *std::min_element(loopTimes.begin(), loopTimes.end()) << "\n"; 

        plot(loopTimes); 
        xlabel(" loops []");
        ylabel("loop times [ms]"); 
        show(); 
    }
}