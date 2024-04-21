#!/bin/bash

echo "Start of tests"

file="results.csv"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

echo "Running tests for sequence 6, stereo, 150 points no loop closer"

for i in 1 2 .. 10
do
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06 0 0 150 0
done 

mv results.csv results_stereo_6_150_no_loop.csv
mv results_stereo_6_150_no_loop.csv ~/dev/projects_cpp/VisualSLAM/results
echo "Done first set of tests "

echo "Running tests for sequence 7, stereo, 150 points no loop closer"
