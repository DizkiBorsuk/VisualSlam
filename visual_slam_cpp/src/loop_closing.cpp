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

        databse = DBoW3::Database(*vocabulary, false, 0)

        while(true)
        {
            std::unique_lock<std::mutex> lock(loop_closer_mutex);
            map_update.wait(lock);

            cv::Mat descriptors; 

            for()
            {
                //get descriptors from new keyframe
                descriptors.push_back(); 
            }

            databse.add(descriotors); 

            // Map::KeyframesType keyframes = map->GetActiveKeyFrames();
            // Map::LandmarksType landmarks = map->GetActiveMapPoints();

            for(std::size_t i = 0; i < database.size(); i++)
            {

            }

        }
    }

    void LoopCloser::end()
    {
        loop_closer_thread.join(); 
    }
}