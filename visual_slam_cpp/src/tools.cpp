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

            x.push_back(poses.at(i).coeff(0,3)/2);  
            y.push_back(poses.at(i).coeff(2,3)/2);
        }

        plot(gt_x, gt_y); 
        hold(on); 
        plot(x, y, "r--");
        xlabel("x [m]");
        ylabel("y [m]");  
        matplot::legend({"prawdziwa trasa", "estymowana trasa"}); 
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

        std::cout << "mean time = " << mean << "ms \n"; 
        std::cout << "min value = " << *std::min_element(loopTimes.begin(), loopTimes.end()) << "\n"; 

        plot(loopTimes); 
        xlabel(" iteracja []");
        ylabel("czas [ms]"); 
        show(); 
    }

    void calculate_error(std::vector<Eigen::Matrix<double, 3,4>> &poses, 
                   std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &gt_poses)
    {   
        double gt_x, gt_y, gt_z, x, y, z;
        std::vector<double> er_x, er_y, er_z; 

        for(size_t i = 0; i < poses.size(); i++)
        {
            gt_x = gt_poses.at(i).coeff(0,3);  
            gt_y = gt_poses.at(i).coeff(2,3);
            gt_z = gt_poses.at(i).coeff(1,3);

            x = poses.at(i).coeff(0,3)/2;  
            y = poses.at(i).coeff(2,3)/2;
            z = poses.at(i).coeff(1,3)/2;

            er_x.emplace_back((gt_x - x)); 
            er_y.emplace_back((gt_y - y));
            er_z.emplace_back((gt_z - z));
        } 

        double sum_error_x = std::accumulate(er_x.begin(), er_x.end(), 0.0); 
        double sum_error_y = std::accumulate(er_y.begin(), er_y.end(), 0.0); 
        double sum_error_z = std::accumulate(er_z.begin(), er_z.end(), 0.0); 

        double mean_error_x = sum_error_x/er_x.size(); 
        double mean_error_y = sum_error_y/er_y.size();  
        double mean_error_z = sum_error_z/er_z.size(); 

        std::cout << "mean x error = " << mean_error_x << "\n"; 
        std::cout << "mean y error = " << mean_error_y << "\n";
        std::cout << "mean z error = " << mean_error_z << "\n";

        tiledlayout(3, 1);
        auto ax1 = nexttile();
        plot(er_x); 
        ylabel("błąd [ms]"); 
        xlabel(" iteracja []");

        auto ax2 = nexttile();
        plot(er_y); 
        ylabel("błąd [ms]"); 
        xlabel(" iteracja []");

        auto ax3 = nexttile();
        plot(er_z); 
        ylabel("błąd [ms]"); 
        xlabel(" iteracja []");
        show(); 

    }
}