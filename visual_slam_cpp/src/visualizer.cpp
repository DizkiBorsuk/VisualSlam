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
          
        std::cout << "------------------------- \n"; 
        std::cout << "visualizer ends work \n"; 
        std::cout << "------------------------- \n";
    }

    void Visualizer::runVisualizer()
    {
        //* main visualizer function run by thread 

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
                drawFrame(current_frame, green); 
                drawFrameTrajectory(s_cam); 

                cv::imshow("current frame", current_frame->imgLeft); 
                cv::waitKey(1); 
            }

            drawPoints(red); 

            // Swap frames and Process Events
            pangolin::FinishFrame();
            std::this_thread::sleep_for(5000ms); // https://en.cppreference.com/w/cpp/thread/sleep_for //? 
        }
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
        displayed_keyframes = map->getEnabledKeyframes(); 
        displayed_mappoints = map->getEnabledMappoints(); 
    }


    void Visualizer::drawFrameTrajectory(pangolin::OpenGlRenderState& s_cam)
    {
        // get Twc - camera pose in world reference transformation matrix and use Pangolin follow  
        //? https://github.com/raulmur/ORB_SLAM2/issues/226
        Sophus::SE3d CameraToWorldTransformation = current_frame->framePose.inverse(); //Tcw 
        pangolin::OpenGlMatrix matrix(CameraToWorldTransformation.matrix()); 
        s_cam.Follow(matrix, true); 
    }

    void Visualizer::drawFrame(std::shared_ptr<Frame> input_frame, const std::array<float,3> color)
    {   
        Sophus::SE3d CameraToWorldTransformation = current_frame->framePose.inverse(); //Tcw 
        
        
        glPushMatrix(); 
        glMultMatrixd((GLdouble*)CameraToWorldTransformation.data()); 

        glColor3f(color[0], color[1], color[2]); 

        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f((0 - cx) / fx, (0 - cy) / fy, 1.0);
        glVertex3f(0, 0, 0);
        
        glVertex3f( (0 - cx) / fx, (window_h - 1 - cy) / fy, 1.0);
        glVertex3f(0, 0, 0);
        glVertex3f((window_w - 1 - cx) / fx, (window_h - 1 - cy) / fy, 1.0);
        glVertex3f(0, 0, 0);
        glVertex3f((window_w - 1 - cx) / fx, (0 - cy) / fy, 1.0);

        glVertex3f((window_w - 1 - cx) / fx, (0 - cy) / fy, 1.0);
        glVertex3f((window_w - 1 - cx) / fx, (window_h - 1 - cy) / fy, 1.0);

        glVertex3f((window_w - 1 - cx) / fx, (window_h - 1 - cy) / fy, 1.0);
        glVertex3f((0 - cx) / fx, (window_h - 1 - cy) / fy, 1.0);

        glVertex3f((0 - cx) / fx, (window_h - 1 - cy) / fy, 1.0);
        glVertex3f((0 - cx) / fx, (0 - cy) / fy, 1.0);

        glVertex3f((0 - cx) / fx, (0 - cy) / fy, 1.0);
        glVertex3f((window_w - 1 - cx) / fx, (0 - cy) / fy, 1.0);

        glEnd();
        glPopMatrix();
    }

    void Visualizer::drawPoints(const std::array<float,3> color)
    {
        for(auto &keyframe : displayed_keyframes)
        {
            drawFrame(keyframe.second, color);  
        }

        glPointSize(2); 
        glBegin(GL_POINTS); 

        for(auto &point : displayed_mappoints)
        {
            auto point_pos = point.second->getPosition(); 
            glColor3f(color[0], color[1], color[2]);
            glVertex3d(point_pos[0], point_pos[1], point_pos[2]);
        }
        glEnd(); 
    }
}