#include "../include/visualizer.hpp"

namespace mrVSLAM
{
    //? https://stackoverflow.com/questions/64743308/pangolin-s-cam-vs-d-cam-need-help-understanding-the-difference

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
        //* main visualizer function run by thread 
        int window_w = 640; 
        int window_h= 480; 

        pangolin::CreateWindowAndBind("slam map",window_w,window_h);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(window_w,window_h,420,420,320,240,0.1,1000),
            pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
        );

          // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, 640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


        while( !pangolin::ShouldQuit() )
        {
            // Clear entire screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    

            if( pangolin::Pushed(a_button) )
            std::cout << "You Pushed a button!" << std::endl;

            // Overloading of Var<T> operators allows us to treat them like
            // their wrapped types, eg:
            if( a_checkbox )
            an_int = (int)a_double;

            an_int_no_input = an_int;

            if(d_cam.IsShown()) {
                // Activate efficiently by object
                d_cam.Activate(s_cam);

                // Render some stuff
                glColor3f(1.0,1.0,1.0);
                pangolin::glDrawColouredCube();
            }

            // Swap frames and Process Events
            pangolin::FinishFrame();
        }
  
        
    }
    

    //######### 

    void Visualizer::addNewFrame(std::shared_ptr<Frame> in_frame)
    {
        std::lock_guard<std::mutex> lock(visualizer_mutex); 
        current_frame = in_frame; 
    }



}