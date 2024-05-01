#!/bin/bash

echo "Start of tests"

file="results.csv"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

echo "Running tests for sequence 6, stereo, 150 points no loop closer"

for i in {0..5}; do
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06 0 0 150 0
done 

mv results.csv results_stereo_6_150_no_loop.csv
mv results_stereo_6_150_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 7, stereo, 150 points no loop closer"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

for i in {0..5}; do
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/07 0 0 150 0
done 

mv results.csv results_stereo_7_150_no_loop.csv
mv results_stereo_7_150_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done second set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 0, stereo, 150 points no loop closer"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

for i in {0..5}; do
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/00 0 0 150 0
done 

mv results.csv results_stereo_0_150_no_loop.csv
mv results_stereo_0_150_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done third set of tests "


# --------------------------------------------------- #

file="results.csv"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

echo "Running tests for sequence 6, stereo, 300 points no loop closer"

for i in {0..5}; do
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06 0 0 300 0
done 

mv results.csv results_stereo_6_300_no_loop.csv
mv results_stereo_6_300_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done fourth set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 7, stereo, 300 points no loop closer"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

for i in {0..5}; do
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/07 0 0 300 0
done 

mv results.csv results_stereo_7_300_no_loop.csv
mv results_stereo_7_300_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done second set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 0, stereo, 300 points no loop closer"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

for i in {0..5}; do
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/00 0 0 300 0
done 

mv results.csv results_stereo_0_300_no_loop.csv
mv results_stereo_0_300_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done third set of tests "

# --------------------------------------------------- #

