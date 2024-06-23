#!/bin/bash

echo "Start of tests"

file="results.csv"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

echo "Running tests for sequence 6, stereo, 400 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i  
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06 0 0 0 400 120 0
    mv map_trajectory.nc results_stereo_gftt_6_400_120_no_loop_${i}.nc 
    mv results_stereo_gftt_6_400_120_no_loop_${i}.nc   ~/dev/projects_cpp/VisualSlam/results/
done 

mv results.csv results_stereo_gftt_6_400_120_no_loop_.csv
mv results_stereo_gftt_6_400_120_no_loop_.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 6, stereo, 400 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i  
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06 0 1 0 400 120 0
    mv map_trajectory.nc results_stereo_orb_6_400_120_no_loop_${i}.nc 
    mv results_stereo_orb_6_400_120_no_loop_${i}.nc   ~/dev/projects_cpp/VisualSlam/results/
done 

mv results.csv results_stereo_orb_6_400_120_no_loop_.csv
mv results_stereo_orb_6_400_120_no_loop_.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 6, stereo, 400 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i  
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06 0 2 0 400 120 0
    mv map_trajectory.nc results_stereo_sift_6_400_120_no_loop_${i}.nc 
    mv results_stereo_sift_6_400_120_no_loop_${i}.nc   ~/dev/projects_cpp/VisualSlam/results/
done 

mv results.csv results_stereo_sift_6_400_120_no_loop_.csv
mv results_stereo_sift_6_400_120_no_loop_.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "

# --------------------------------------------------- #
#----------------------------------------------------# 

echo "Running tests for sequence 6, stereo, 400 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i  
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/07 0 0 0 400 120 0
    mv map_trajectory.nc results_stereo_gftt_7_400_120_no_loop_${i}.nc 
    mv results_stereo_gftt_7_400_120_no_loop_${i}.nc   ~/dev/projects_cpp/VisualSlam/results/
done 

mv results.csv results_stereo_gftt_7_400_120_no_loop_.csv
mv results_stereo_gftt_7_400_120_no_loop_.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 7, stereo, 400 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i  
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/07 0 1 0 400 120 0
    mv map_trajectory.nc results_stereo_orb_7_400_120_no_loop_${i}.nc 
    mv results_stereo_orb_7_400_120_no_loop_${i}.nc   ~/dev/projects_cpp/VisualSlam/results/
done 

mv results.csv results_stereo_orb_7_400_120_no_loop_.csv
mv results_stereo_orb_7_400_120_no_loop_.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 7, stereo, 400 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i  
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/07 0 2 0 400 120 0
    mv map_trajectory.nc results_stereo_sift_7_400_120_no_loop_${i}.nc 
    mv results_stereo_sift_7_400_120_no_loop_${i}.nc   ~/dev/projects_cpp/VisualSlam/results/
done 

mv results.csv results_stereo_sift_7_400_120_no_loop_.csv
mv results_stereo_sift_7_400_120_no_loop_.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "

# --------------------------------------------------- #
# --------------------------------------------------- #

echo "Running tests for sequence 6, stereo, 400 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i  
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/00 0 0 0 400 120 0
    mv map_trajectory.nc results_stereo_gftt_0_400_120_no_loop_${i}.nc 
    mv results_stereo_gftt_0_400_120_no_loop_${i}.nc   ~/dev/projects_cpp/VisualSlam/results/
done 

mv results.csv results_stereo_gftt_0_400_120_no_loop_.csv
mv results_stereo_gftt_0_400_120_no_loop_.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 0, stereo, 400 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i  
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/00 0 1 0 400 120 0
    mv map_trajectory.nc results_stereo_orb_0_400_120_no_loop_${i}.nc 
    mv results_stereo_orb_0_400_120_no_loop_${i}.nc   ~/dev/projects_cpp/VisualSlam/results/
done 

mv results.csv results_stereo_orb_0_400_120_no_loop_.csv
mv results_stereo_orb_0_400_120_no_loop_.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 0, stereo, 400 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i  
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/00 0 2 0 400 120 0
    mv map_trajectory.nc results_stereo_sift_0_400_120_no_loop_${i}.nc 
    mv results_stereo_sift_0_400_120_no_loop_${i}.nc   ~/dev/projects_cpp/VisualSlam/results/
done 

mv results.csv results_stereo_sift_0_400_120_no_loop_.csv
mv results_stereo_sift_0_400_120_no_loop_.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "