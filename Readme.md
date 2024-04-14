# Visual SLAM project 

This is a project I did as a part of my master thesis and my intrest in SLAM. It's a Visual SLAM that uses mono or stereo camera. 

## How to install/compile 
### Dependecies 
This project uses following libs: 
1. Eigen - matrix operation library, similar to matlab, 
2. OpenCV - computer vision library, 
3. Sophus - wrapper around Eigen library that allows easy use of Lie groups i.e transformation matricies, 
4. G2O - library that implements graph optimization, used for bundle adjustment etc., 
5. Boost - boost.config for easy data reading 
6. Pangolin - visualization library based on OpenGL, popular in SLAM, 
7. DBoW/FBoW 
8. FMT - logging library, 
9. Matplot++

## How to run
In visual_slam_cpp directory: 
- With default parameters: ./run_SLAM 
- With custom parameters: ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/07 


## Libraries that i used or are useful in SLAM project  
### C++ 
1. Eigen - matrix operation library, similar to matlab, 
2. OpenCV - can't do computer vision without opencv, 
3. Pangolin - library for SLAM visualization based on OpneGL,  
4. Ceres / G2O/ GTSAM - optimization libraries
5. matplotlibcpp/matplot++ 
6. DBoW3/DBoW2 - bag of words for loop closing, BDoW3 is update to famous DBoW2, i made some changes to DBoW3 to make it work in C++17 and later
7. Open3D 

### Python 
1. OpenCV 
2. Numpy, Pandas 
3. Scikit / Scikit-image
4. G2O
5. Pangolin


## Useful materials, theory etc. 

Probabilistic Robotics - Sebastian Thurn <br />
Cyrill Stachniss yt channel - https://www.youtube.com/@CyrillStachniss <br />
MonoSLAM: Real-Time Single Camera SLAM <br />
OrbSLAM articles <br />
https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html - good opencv documentation, old but all the functions are preaty much the same <br/>
https://nl.mathworks.com/help/vision/visual-simultaneous-localization-and-mapping-slam.html<br />
https://towardsdatascience.com/image-feature-extraction-traditional-and-deep-learning-techniques-ccc059195d04 <br />
https://www.kudan.io/blog/visual-slam-the-basics/ <br/>
https://www.kudan.io/blog/direct-visual-slam/ <br />



