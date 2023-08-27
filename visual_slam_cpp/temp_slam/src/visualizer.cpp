#include "../include/visualizer.hpp"

using namespace std::chrono_literals;

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
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(window_w,window_h,420,420,320,240,0.1,1000),
            pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisNone)
        );

          // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0, 1.0, 640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


        while( !pangolin::ShouldQuit() )
        {
            // Clear entire screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            if(d_cam.IsShown()) {
                // Activate efficiently by object
                d_cam.Activate(s_cam);
            }

            std::lock_guard<std::mutex> lock(visualizer_mutex); 

            if(current_frame != nullptr)
            {
                drawFrame(current_frame); 
                drawFrameTrajectory(s_cam); 

                cv::imshow("current frame", current_frame->imgLeft); 
                cv::waitKey(1); 
            }

            // Swap frames and Process Events
            pangolin::FinishFrame();
            std::this_thread::sleep_for(5000ms); // https://en.cppreference.com/w/cpp/thread/sleep_for //? 
        }
  
        std::cout << "------------------------- \n"; 
        std::cout << "visualizer ends work \n"; 
        std::cout << "------------------------- \n";
    }
    

    //######### 

    void Visualizer::addNewFrame(std::shared_ptr<Frame> in_frame)
    {
        std::lock_guard<std::mutex> lock(visualizer_mutex); 
        current_frame = in_frame; 
    }

    void Visualizer::getMapUpdate()
    {
        std::lock_guard<std::mutex> lock(visualizer_mutex); 
    }


    void Visualizer::drawFrameTrajectory(pangolin::OpenGlRenderState& s_cam)
    {
        // get Twc - camera pose in world reference transformation matrix and use Pangolin follow  
        //? https://github.com/raulmur/ORB_SLAM2/issues/226
        Eigen::Matrix4d CameraToWorldTransformation = current_frame->framePose.inv(); //Tcw //! find solution or start using eigen 
        pangolin::OpenGlMatrix matrix(CameraToWorldTransformation); 
        s_cam.Follow(matrix, true); 
    }

    void Visualizer::drawFrame()
    {
        
    }



}