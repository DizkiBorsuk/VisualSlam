#include "myslam/viewer.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std::chrono_literals;

namespace myslam {

    Visualizer::Visualizer(bool show_whole_map) 
    {
        whole_map = show_whole_map; 
        visualizer_thread = std::thread(std::bind(&Visualizer::ThreadLoop, this));
    }

    void Visualizer::Close() 
    {
        visualizer_running = false;
        visualizer_thread.join();
    }

    void Visualizer::AddCurrentFrame(std::shared_ptr<Frame> current_frame) 
    {
        std::unique_lock<std::mutex> lock(visualizer_mutex);
        current_frame_ = current_frame;
    }

    void Visualizer::UpdateMap() 
    {
        std::unique_lock<std::mutex> lock(visualizer_mutex);
        assert(map != nullptr);
        if(whole_map == true)
        {
            keyframes = map->GetAllKeyFrames();
            landmarks = map->GetAllMapPoints();
        } else {
            keyframes = map->GetActiveKeyFrames();
            landmarks = map->GetActiveMapPoints();
        }

        map_updated = true;
    }

    void Visualizer::ThreadLoop() 
    {
        pangolin::CreateWindowAndBind("slam visualizer", width, height);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera(
            pangolin::ProjectionMatrix(width, height, 400, 400, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& vis_display =
            pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(vis_camera));

        while (!pangolin::ShouldQuit() && visualizer_running) 
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            vis_display.Activate(vis_camera);

            std::unique_lock<std::mutex> lock(visualizer_mutex);
            if (current_frame_) {
                DrawFrame(current_frame_, green);
                FollowCurrentFrame(vis_camera);

                cv::Mat img = PlotFrameImage();
                cv::imshow("kitti img", img);
                cv::waitKey(1);
            }
        
            DrawMapPoints(blue);

            pangolin::FinishFrame();
            // usleep(5000);
            std::this_thread::sleep_for(5000us); 
        }

        std::cout << "Stop viewer \n";
    }

    cv::Mat Visualizer::PlotFrameImage() 
    {
        cv::Mat img_out;
        cv::cvtColor(current_frame_->left_img_, img_out, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            if (current_frame_->features_left_[i]->map_point_.lock()) {
                auto feat = current_frame_->features_left_[i];
                cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0), 2);
            }
        }
        return img_out;
    }

    void Visualizer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) 
    {
        Sophus::SE3d Twc = current_frame_->Pose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
        vis_camera.Follow(m, true);
    }

    void Visualizer::DrawFrame(std::shared_ptr<Frame> frame, const int* frame_color) 
    {
        Sophus::SE3d Twc = frame->Pose().inverse();

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

    void Visualizer::DrawMapPoints(const int* point_color)
    {
        for (auto& kf : keyframes) {
            DrawFrame(kf.second, red);
        }

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto& landmark : landmarks) {
            auto pos = landmark.second->Pos();
            glColor3f(point_color[0], point_color[1], point_color[2]);
            glVertex3d(pos[0], pos[1], pos[2]);
        }
        glEnd();
    }

}  