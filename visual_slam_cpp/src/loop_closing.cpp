#include "myslam/loop_closing.hpp"

namespace myslam
{
    LoopClosing::LoopClosing()
    {
        loop_closer_thread = std::thread(std::bind(&LoopClosing::runLoopCloser, this)); 
    }

    void LoopClosing::runLoopCloser()
    {
        // vocabulary = DBoW3::Vocabulary("vocab_file"); 
        // vocabulary.transform(); // transforms features into words 
    }
}