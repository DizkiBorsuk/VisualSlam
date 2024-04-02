/**
 * @file visualizer.cpp
 * @author mrostocki
 * @brief 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "mrVSLAM/visualizer.hpp" 
#include "mrVSLAM/frame.hpp"
#include "mrVSLAM/map.hpp"
#include "mrVSLAM/mappoint.hpp"

namespace mrVSLAM
{
    Visualizer::Visualizer(bool show_whole_map, bool show_img, bool show_matching_points)
    {
        this->whole_map = show_whole_map; 
        this->show_img = show_img; 
        this->show_matching_points = show_matching_points; 
        this->visualizer_thread = std::thread(std::bind(&Visualizer::runVisualizerThread, this)); 
        fmt::print(fg(fmt::color::aqua), "visualizer thread started \n"); 
    }

    void Visualizer::close()
    {
        visualizer_running = false; 
        visualizer_thread.join(); 
        fmt::print(bg(fmt::color::indian_red), "visualizer thread closed \n"); 
    }

    bool Visualizer::updateMap()
    {
        std::unique_lock<std::mutex> lock(visualizer_mutex);
        assert(map != nullptr);
        if(whole_map == true)
        {
            keyframes = map->getAllKeyframes();
            mappoints = map->getAllMappoints();
        } else {
            keyframes = map->getActiveKeyframes();
            mappoints = map->getActiveMappoints();
        }
        return true; 
    }

    /**
     * @brief create visualization of camera and it's projection in map 
     * 
     * @param frame ptr to frame object 
     * @param frame_color - color in which to draw
     */
    void Visualizer::drawFrame(std::shared_ptr<Frame> frame, const int* frame_color)
    {
        Sophus::SE3d Twc = frame->getPose().inverse();

        glPushMatrix();

        Eigen::Matrix4f Tmatrix = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat*)Tmatrix.data());

        if (frame_color == nullptr) {
            glColor3f(1, 0, 0);
        } else
            glColor3f(frame_color[0], frame_color[1], frame_color[2]);

        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();
    }

    /**
     * @brief draw all frames and mappoints that are stored in selected hash tables 
     * @param point_color - color of points in map 
     */
    void Visualizer::drawMap(const int* point_color)
    {
        for (auto& kf : keyframes) 
        {
            drawFrame(kf.second, red);
        }

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto& landmark : mappoints) {
            auto pos = landmark.second->getPointPosition();
            glColor3f(point_color[0], point_color[1], point_color[2]);
            glVertex3d(pos[0], pos[1], pos[2]);
        }
        glEnd();
    }

    /**
     * @brief move map point of view in a way that follows current frame 
     * @param vis_camera 
     */
    void Visualizer::followNewFrame(pangolin::OpenGlRenderState& vis_camera)
    {
        Sophus::SE3d Twc = current_frame->getPose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
        vis_camera.Follow(m, true);
    }

    cv::Mat Visualizer::drawFrameImg(bool draw_matching_points)
    {
        cv::Mat processed_img = cv::Mat(current_frame->left_img);
        cv::cvtColor(processed_img, processed_img, cv::COLOR_GRAY2BGR); 

        for(size_t i = 0; i < current_frame->features_on_left_img.size(); i++)
        {
            if(current_frame->features_on_left_img.at(i)->map_point.lock())
            {
                auto feature = current_frame->features_on_left_img.at(i); 
                cv::circle(processed_img, feature->positionOnImg.pt, 2, cv::Scalar(0,250,0),2); 
            }

            if(draw_matching_points && prev_frame) //! would need to match points of find way to match points based on mappoint 
            {
                // cv::circle(frame, features.matched_keypoints[p][0], 3, cv::Scalar(0,255,0));
                // cv::line(frame, features.matched_keypoints[p][1], features.matched_keypoints[p][0], cv::Scalar(255,0,0), 1); 
            }

        }

        return processed_img; 
    }

    /**
     * @brief main visualizer function that creates map and img visualization in loop 
     * @details 
     */
    void Visualizer::runVisualizerThread()
    {
        pangolin::CreateWindowAndBind("slam visualizer", width, height);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera(
            pangolin::ProjectionMatrix(width, height, 400, 400, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& vis_display = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -width/height).SetHandler(new pangolin::Handler3D(vis_camera));

        //* Main Visualizer Loop 
        while (!pangolin::ShouldQuit() && visualizer_running) 
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            vis_display.Activate(vis_camera);

            std::unique_lock<std::mutex> lock(visualizer_mutex);
            if (current_frame) 
            {
                drawFrame(current_frame, green);
                followNewFrame(vis_camera);

                if(show_img)
                {
                    cv::Mat img = drawFrameImg();
                    cv::imshow("kitti img", img);
                    cv::waitKey(1);
                }
            }
        
            drawMap(blue);

            pangolin::FinishFrame();
            std::this_thread::sleep_for(5000us); 
            prev_frame = current_frame; 
        }

        std::cout << "Stop viewer \n";
    }

} //! end of namespace