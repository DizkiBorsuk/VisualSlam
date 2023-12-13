#include "myslam/tools.hpp"
#include <matplot/matplot.h>
#include <cmath>
#include <functional>
#include <execution>
#include "myslam/frame.hpp"

#define PAR std::execution::par,

namespace myslam
{

    void plotPoses(std::vector<Eigen::Matrix<double, 3,4>> &poses, std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &gt_poses, float resize_opt)
    {
        std::vector<double> gt_x, gt_y, x, y; 
 
        for(std::size_t i = 0; i < poses.size(); i++)
        {
            gt_x.push_back(gt_poses.at(i).coeff(0,3));  
            gt_y.push_back(gt_poses.at(i).coeff(2,3));

            x.push_back(poses.at(i).coeff(0,3)*resize_opt);  
            y.push_back(poses.at(i).coeff(2,3)*resize_opt);
        }

        auto fig = matplot::figure();  
        fig->width(fig->width()*1.3); 
        fig->height(fig->height()*1.3);
        fig->color("white"); 
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

        auto fig = matplot::figure();  
        fig->width(fig->width()*1.3); 
        fig->height(fig->height()*1.3);
        fig->color("white"); 
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

        auto fig = matplot::figure();  
        fig->width(fig->width()*1.3); 
        fig->height(fig->height()*1.3);
        fig->color("white");  
        matplot::plot(loopTimes); 
        matplot::xlabel(" iteracja []");
        matplot::ylabel("czas [ms]"); 
        matplot::show(); 
    }

    void calculate_error(std::vector<Eigen::Matrix<double, 3,4>> &poses, std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &gt_poses, float resize_opt, int seq)
    {   
        double gt_x, gt_y, gt_z, x, y, z;
        

        std::vector<double> er_x, er_y, er_z, er_x1, er_y1, er_z1; 

        for(size_t i = 0; i < poses.size(); i++)
        {
            gt_x = gt_poses.at(i).coeff(0,3);  
            gt_y = gt_poses.at(i).coeff(2,3);
            gt_z = gt_poses.at(i).coeff(1,3);

            x = poses.at(i).coeff(0,3)*resize_opt;  
            y = poses.at(i).coeff(2,3)*resize_opt;
            z = poses.at(i).coeff(1,3)*resize_opt;

            er_x.emplace_back((gt_x - x)*(gt_x - x)); 
            er_y.emplace_back((gt_y - y)*(gt_y - y));
            er_z.emplace_back((gt_z - z)*(gt_z - z));

            er_x1.emplace_back((gt_x - x)); 
            er_y1.emplace_back((gt_y - y));
            er_z1.emplace_back((gt_z - z));
        } 


        // auto abs_value = [](auto value) {return std::abs(value); }; 
        auto compare_abs = [](auto value1, auto value2) {return std::abs(value1) < std::abs(value2);}; 

        // double sum_error_x = std::transform_reduce(PAR er_x.cbegin(), er_x.cend(), 0L, std::plus{}, abs_value);
        // double sum_error_y = std::transform_reduce(PAR er_y.cbegin(), er_y.cend(), 0L, std::plus{}, abs_value);
        // double sum_error_z = std::transform_reduce(PAR er_z.cbegin(), er_z.cend(), 0L, std::plus{}, abs_value);

        double sum_error_x = std::accumulate(er_x.begin(), er_x.end(), 0);
        double sum_error_y = std::accumulate(er_y.begin(), er_y.end(), 0); 
        double sum_error_z = std::accumulate(er_z.begin(), er_z.end(), 0);

        double mean_error_x = sqrt(sum_error_x/er_x.size()); 
        double mean_error_y = sqrt(sum_error_y/er_y.size());  
        double mean_error_z = sqrt(sum_error_z/er_z.size()); 
        double mean_e = (mean_error_x + mean_error_y + mean_error_z)/3; 
        double percent_e = 0; 

        double distance = 0;

        switch (seq)
        {
        case 0: 
            // distance = 
            break; 
        case 6:
            distance = 1232.87; 
            percent_e = (mean_e/distance)*100; 
            break;
        case 7:
            distance = 649.69; 
            percent_e = (mean_e/distance)*100; 
            break;
        
        default:
            break;
        }

        std::cout << "mean error = " << mean_e << ", "<< percent_e <<"\n";
        std::cout << "mean x error = " << mean_error_x << "\n";
        std::cout << "max x error = " << *std::max_element(er_x.begin(), er_x.end(), compare_abs) << ", min x error = " << *std::min_element(er_x.begin(), er_x.end(), compare_abs) << "\n"; 
        std::cout << "mean y error = " << mean_error_y << "\n";
        std::cout << "max y error = " << *std::max_element(er_y.begin(), er_y.end(), compare_abs) << ", min y error = " << *std::min_element(er_y.begin(), er_y.end(), compare_abs) << "\n"; 
        std::cout << "mean z error = " << mean_error_z << "\n";
        std::cout << "max z error = " << *std::max_element(er_z.begin(), er_z.end(), compare_abs) << ", min z error = " << *std::min_element(er_z.begin(), er_z.end(),compare_abs) << "\n"; 

        for(size_t i =0; i < er_x.size(); i++)
        {
            er_x.at(i) = sqrt(er_x.at(i));
            er_y.at(i) = sqrt(er_y.at(i));
            er_z.at(i) = sqrt(er_z.at(i));
        }

        auto fig = matplot::figure();  
        fig->width(fig->width()*2); 
        fig->height(fig->height()*2);
        fig->color("white"); 
        auto ax1 = matplot::subplot(3, 1, 0);
        matplot::plot(er_x1)->line_width(2);
        matplot::grid(ax1, matplot::on);   
        matplot::ylabel("błąd x [m]"); 
        matplot::xlabel(" iteracja []");

        auto ax2 = matplot::subplot(3, 1, 1);
        matplot::plot(er_y1)->line_width(2); 
        matplot::grid(ax2, matplot::on);  
        matplot::ylabel("błąd y [m]"); 
        matplot::xlabel(" iteracja []");

        auto ax3 = matplot::subplot(3, 1, 2);
        matplot::plot(er_z1)->line_width(2);
        matplot::grid(ax3, matplot::on);   
        matplot::ylabel("błąd z [m]"); 
        matplot::xlabel(" iteracja []");
        fig->draw(); 
        matplot::show(); 

    }

    void plotPosesWitLoopPairs(std::vector<Eigen::Matrix<double, 3,4>> &poses, std::vector<std::array<std::shared_ptr<Frame>, 2>> kf_pairs)
    {
        std::vector<double>  x, y; 
        std::vector<unsigned int> kf_id1, kf_id2; 
 
        for(std::size_t i = 0; i < poses.size(); i++)
        {
            x.push_back(poses.at(i).coeff(0,3));  
            y.push_back(poses.at(i).coeff(2,3));
            
        }


        auto fig = matplot::figure();  


        for(std::size_t i = 0; i < kf_pairs.size(); i++)
        {
            std::vector<double> x1, y1, x2, y2, x_c, y_c; 
            std::cout << "loop candidate 1 = " << kf_pairs.at(i).at(0)->keyframe_id << ", loop candidate 2 = " << kf_pairs.at(i).at(1)->keyframe_id<< "\n"; 
    
            x1.emplace_back(kf_pairs.at(i).at(0)->getPose().inverse().matrix3x4().coeff(0,3)); 
            y1.emplace_back(kf_pairs.at(i).at(0)->getPose().inverse().matrix3x4().coeff(2,3));

            x2.emplace_back(kf_pairs.at(i).at(1)->getPose().inverse().matrix3x4().coeff(0,3)); 
            y2.emplace_back(kf_pairs.at(i).at(1)->getPose().inverse().matrix3x4().coeff(2,3));

            std::cout << "x1,y1 = " << x1.at(0) << "," << y1.at(0) << " x2,y2 = " << x2.at(0) << "," << y2.at(0) << "\n"; 
            x_c.emplace_back(x1.at(0)); 
            x_c.emplace_back(x2.at(0)); 
            y_c.emplace_back(y1.at(0)); 
            y_c.emplace_back(y2.at(0)); 
            
            matplot::plot(x, y, "b")->line_width(2);
            matplot::xlabel("x [m]");
            matplot::ylabel("y [m]");  
            matplot::hold(matplot::on);
            auto sc1 = matplot::scatter(x1, y1, 10); 
            sc1->marker_style(matplot::line_spec::marker_style::asterisk); 
            auto sc2 = matplot::scatter(x2, y2, 10); 
            sc2->marker_style(matplot::line_spec::marker_style::circle); 
            matplot::plot(x_c,y_c, "k"); 
            matplot::show();
   
        }
        

        // auto fig = matplot::figure();  
        // fig->width(fig->width()*1.3); 
        // fig->height(fig->height()*1.3);
        // fig->color("white"); 
        // //matplot::plot(x, y, "r")->line_width(2);
        // matplot::hold(matplot::on); 
        // matplot::plot(kf_id1, "g*"); 
        // matplot::plot(kf_id2, "b*"); 
        // matplot::xlabel("x [m]");
        // matplot::ylabel("y [m]");  
        // matplot::show();
    }


}