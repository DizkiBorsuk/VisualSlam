#include "../include/visualizer.hpp"

namespace mrVSLAM
{

    Visualizer::Visualizer()
    {
        // create thread and run visualizer 
        visualizer_thread = std::thread(std::bind(&Visualizer::runVisualizer, this)); //? change it to lambda   
    }

    void Visualizer::closeVisualizer() 
    {
        visualizer_thread.join(); //? maybe change to pthread 
    }

    void Visualizer::runVisualizer()
    {
        // main visualizer function run by thread 
        
    }



}