#include "myslam/loop_closing.hpp"
#include "myslam/frame.hpp"

namespace myslam
{
    LoopClosing::LoopClosing(std::shared_ptr<DBoW3::Vocabulary> vocab)
    {
        database = DBoW3::Database(*vocab, false, 0); 
        loop_closer_thread = std::thread(std::bind(&LoopClosing::runLoopCloser, this)); 
    }

    void LoopClosing::runLoopCloser()
    {


        while(true)
        {
            // std::unique_lock<std::mutex> lock(loop_closer_mutex);
            // map_update.wait(lock);

            // database.add(current_frame->bow_vector); 

            // // Map::KeyframesType keyframes = map->GetActiveKeyFrames();
            // // Map::LandmarksType landmarks = map->GetActiveMapPoints();

            // for(std::size_t i = 0; i < database.size(); i++)
            // {

            // }

        }
    }

    void LoopClosing::end()
    {
        loop_closer_thread.join(); 
    }
}