#pragma once 
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudafeatures2d.hpp>

namespace mrVSLAM
{
    class VO
    {
    public: 

    void detectFeatures(cv::Mat frame, const std::string& descriptor);  //change descriptor to enum? 

    }; 




}