#include "myslam/tools.hpp"
#include <matplot/matplot.h>
#include <cmath>
#include <functional>
#include <execution>

#define PAR std::execution::par,

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

        matplot::plot(gt_x, gt_y)->line_width(2); 
        matplot::hold(matplot::on); 
        matplot::plot(x, y, "r--")->line_width(2);
        matplot::xlabel("x [m]");
        matplot::ylabel("y [m]");  
        matplot::legend({"prawdziwa trasa", "estymowana trasa"}); 
        matplot::show();
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

        matplot::plot(x, y, "r--")->line_width(2);
        matplot::xlabel("x [m]");
        matplot::ylabel("y [m]");
        matplot::show();
    }
    void plotPerformance(std::vector<int> loopTimes)
    {

        int sum = std::accumulate(loopTimes.begin(), loopTimes.end(), 0.0); 
        double mean = sum/loopTimes.size(); 

        std::cout << "mean time = " << mean << "ms \n"; 
        std::cout << "min value = " << *std::min_element(loopTimes.begin(), loopTimes.end()) << "\n"; 

        matplot::plot(loopTimes); 
        matplot::xlabel(" iteracja []");
        matplot::ylabel("czas [ms]"); 
        matplot::show(); 
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

        auto abs_value = [](auto value)
        {
            return std::abs(value);
        }; 
        auto compare_abs = [](auto value1, auto value2)
        {
            return std::abs(value1) < std::abs(value2);
        }; 

        double sum_error_x = std::transform_reduce(PAR er_x.cbegin(), er_x.cend(), 0L, std::plus{}, abs_value);
        double sum_error_y = std::transform_reduce(PAR er_x.cbegin(), er_x.cend(), 0L, std::plus{}, abs_value);
        double sum_error_z = std::transform_reduce(PAR er_x.cbegin(), er_x.cend(), 0L, std::plus{}, abs_value);

        double mean_error_x = sum_error_x/er_x.size(); 
        double mean_error_y = sum_error_y/er_y.size();  
        double mean_error_z = sum_error_z/er_z.size(); 

        std::cout << "mean x error = " << mean_error_x << "\n";
        std::cout << "max x error = " << *std::max_element(er_x.begin(), er_x.end(), compare_abs) << ", min x error = " << *std::min_element(er_x.begin(), er_x.end(), compare_abs) << "\n"; 
        std::cout << "mean y error = " << mean_error_y << "\n";
        std::cout << "max y error = " << *std::max_element(er_y.begin(), er_y.end(), compare_abs) << ", min y error = " << *std::min_element(er_y.begin(), er_y.end(), compare_abs) << "\n"; 
        std::cout << "mean z error = " << mean_error_z << "\n";
        std::cout << "max z error = " << *std::max_element(er_z.begin(), er_z.end(), compare_abs) << ", min z error = " << *std::min_element(er_z.begin(), er_z.end(),compare_abs) << "\n"; 
 
        auto fig = matplot::figure();  
        fig->width(fig->width()*2); 
        fig->height(fig->height()*2);
        auto ax1 = matplot::subplot(3, 1, 0);
        matplot::plot(er_x)->line_width(2);
        matplot::grid(ax1, matplot::on);   
        matplot::ylabel("błąd x [m]"); 
        matplot::xlabel(" iteracja []");

        auto ax2 = matplot::subplot(3, 1, 1);
        matplot::plot(er_y)->line_width(2); 
        matplot::grid(ax2, matplot::on);  
        matplot::ylabel("błąd y [m]"); 
        matplot::xlabel(" iteracja []");

        auto ax3 = matplot::subplot(3, 1, 2);
        matplot::plot(er_z)->line_width(2);
        matplot::grid(ax3, matplot::on);   
        matplot::ylabel("błąd z [m]"); 
        matplot::xlabel(" iteracja []");
        fig->draw(); 
        matplot::show(); 

    }
}