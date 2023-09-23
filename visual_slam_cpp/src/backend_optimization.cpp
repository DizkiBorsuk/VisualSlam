#include "../include/backend_optimization.hpp"

namespace mrVSLAM
{
    Backend::Backend() noexcept
    {
        // std::bind generates a call wrapper for function, calling wrapper is like invokeing function //works like function pointer
        backend_thread = std::thread(std::bind(&Backend::runBackend, this)); // start bakcend thread
        std::cout << "started optimization thread \n"; 
    }

    void Backend::setBackend(std::shared_ptr<Map> in_map, std::shared_ptr<Camera> in_camera_left, std::shared_ptr<Camera> in_camera_right)
    {
        map = in_map; 
        camera_left = in_camera_left; 
        camera_right = in_camera_right; 
        std::cout << "backend pointers to map and cameras set \n";   
    }

    void Backend::endBackendThread()
    {
        backend_thread.join(); // join to main thread
        std::cout << "end of optimization thread \n"; 
    }

    void Backend::runBackend()
    {
        
    }


    void Backend::updateMap()
    {
       std::lock_guard<std::mutex> lock(backend_mutex); 
       map_update_var.notify_one(); //https://en.cppreference.com/w/cpp/thread/condition_variable/notify_one
    }
}