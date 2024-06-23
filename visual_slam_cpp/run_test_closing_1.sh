#!/bin/bash

echo "Start of tests"

file="results.csv"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

# --------------------------------------------------- #
# --------------------- GFTT ------------------------ # 
# --------------------------------------------------- #

echo "Running tests for sequence 6, stereo, 400 points loop closer"

for i in 1 2 3 4 5 6 7 8 9 10
do
    echo $i  
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06 0 0 1 400 80 0
    mv map_trajectory.nc results_stereo_gftt_6_400_closing_${i}.nc 
    mv results_stereo_gftt_6_400_closing_${i}.nc   ~/dev/projects_cpp/VisualSlam/results/
done 

mv results.csv results_stereo_gftt_6_400_closing.csv
mv results_stereo_gftt_6_400_closing.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "


