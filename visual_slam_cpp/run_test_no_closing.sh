#!/bin/bash

echo "Start of tests"

file="results.csv"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

echo "Running tests for sequence 0, stereo, 150 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i 
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/00 0 2 0 150 0
    mv map_trajectory.nc results_stereo_sift_0_150_no_loop_${i}.nc 
    mv results_stereo_sift_0_150_no_loop_${i}.nc   ~/dev/projects_cpp/VisualSlam/results/
done 

mv results.csv results_stereo_sift_0_150_no_loop.csv
mv results_stereo_sift_0_150_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done third set of tests "