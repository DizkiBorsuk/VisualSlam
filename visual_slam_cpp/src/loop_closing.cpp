#include "myslam/loop_closing.hpp"
#include "myslam/frame.hpp"

namespace myslam
{
    LoopClosing::LoopClosing(std::shared_ptr<DBoW3::Vocabulary> vocab)
    {
        database = DBoW3::Database(*vocab, false, 0); 
        loop_closer_running.store(true); 
        loop_closer_thread = std::thread(std::bind(&LoopClosing::runLoopCloser, this)); 
    }

    void LoopClosing::addCurrentKeyframe(std::shared_ptr<Frame> new_keyframe)
    {
        /*
        adds current keyframe to keyframe database, then it is compared with every other keyframe.
        */
        std::unique_lock<std::mutex> lock(loop_closer_mutex); 
        current_frame = new_keyframe; 
        map_update.notify_one();
    }


    void LoopClosing::runLoopCloser()
    {
        while(loop_closer_running.load())
        {
            std::unique_lock<std::mutex> lock(loop_closer_mutex);
            map_update.wait(lock);

            database.add(current_frame->bow_vector); 
            DBoW3::QueryResults similarity; 
            int z = 0; 

            if(database.size()>10)
            {
                database.query(current_frame->bow_vector, similarity, 0); 

                for(std::size_t i = 0; i < similarity.size(); i++)
                {
                    if(similarity.at(i).Score > 0.05 && similarity.at(i).Score < 0.99) //&& similarity.at(i).Score < 0.95
                    {
                        std::cout << "keyframe id = " << current_frame->keyframe_id << "\n"; 
                        std::cout << "found loop closing at " << similarity.at(i) << "\n"; 
                    }
                    
                }
                std::cout << "database size " << database.size() << "\n"; 
                std::cout << "compared " << z << " images \n"; 
            }
        }
    }

    void LoopClosing::end()
    {
        loop_closer_running.store(false); 
        map_update.notify_one(); 
        loop_closer_thread.join(); 
    }
}