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

echo "Running tests for sequence 6, stereo, 150 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i  
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06 0 0 0 150 0
done 

mv results.csv results_stereo_gftt_6_150_no_loop.csv
mv results_stereo_gftt_6_150_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 7, stereo, 150 points no loop closer"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

for i in 1 2 3 4 5
do
    echo $i 
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/07 0 0 0 150 0
done 

mv results.csv results_stereo_gftt_7_150_no_loop.csv
mv results_stereo_gftt_7_150_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done second set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 0, stereo, 150 points no loop closer"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

for i in 1 2 3 4 5
do
    echo $i 
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/00 0 0 0 150 0
done 

mv results.csv results_stereo_gftt_0_150_no_loop.csv
mv results_stereo_gftt_0_150_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done third set of tests "


# --------------------------------------------------- #

file="results.csv"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

echo "Running tests for sequence 6, stereo, 300 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i 
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06 0 0 0 400 0
done 

mv results.csv results_stereo_gftt_6_300_no_loop.csv
mv results_stereo_gftt_6_300_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done fourth set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 7, stereo, 300 points no loop closer"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

for i in 1 2 3 4 5
do
    echo $i 
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/07 0 0 0 400 0
done 

mv results.csv results_stereo_gftt_7_300_no_loop.csv
mv results_stereo_gftt_7_300_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done fifth set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 0, stereo, 300 points no loop closer"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

for i in 1 2 3 4 5
do
    echo $i 
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/00 0 0 0 400 0
done 

mv results.csv results_stereo_gftt_0_300_no_loop.csv
mv results_stereo_gftt_0_300_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done sixth set of tests "

# --------------------------------------------------- #

# --------------------------------------------------- #
# --------------------- ORB ------------------------- # 
# --------------------------------------------------- #

echo "Running tests for sequence 6, stereo, 150 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i  
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06 0 1 0 150 0
done 

mv results.csv results_stereo_orb_6_150_no_loop.csv
mv results_stereo_orb_6_150_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done first set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 7, stereo, 150 points no loop closer"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

for i in 1 2 3 4 5
do
    echo $i 
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/07 0 1 0 150 0
done 

mv results.csv results_stereo_orb_7_150_no_loop.csv
mv results_stereo_orb_7_150_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done second set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 0, stereo, 150 points no loop closer"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

for i in 1 2 3 4 5
do
    echo $i 
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/00 0 1 0 150 0
done 

mv results.csv results_stereo_orb_0_150_no_loop.csv
mv results_stereo_orb_0_150_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done third set of tests "


# --------------------------------------------------- #

file="results.csv"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

echo "Running tests for sequence 6, stereo, 300 points no loop closer"

for i in 1 2 3 4 5
do
    echo $i 
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06 0 1 0 400 0
done 

mv results.csv results_stereo_orb_6_300_no_loop.csv
mv results_stereo_orb_6_300_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done fourth set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 7, stereo, 300 points no loop closer"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

for i in 1 2 3 4 5
do
    echo $i 
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/07 0 1 0 400 0
done 

mv results.csv results_stereo_orb_7_300_no_loop.csv
mv results_stereo_orb_7_300_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done fifth set of tests "

# --------------------------------------------------- #

echo "Running tests for sequence 0, stereo, 300 points no loop closer"

if [ -f "$file" ] ; then
    echo "removing old results file"
    rm "$file"
fi

for i in 1 2 3 4 5
do
    echo $i 
    ./run_SLAM ~/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/00 0 1 0 400 0
done 

mv results.csv results_stereo_orb_0_300_no_loop.csv
mv results_stereo_orb_0_300_no_loop.csv ~/dev/projects_cpp/VisualSlam/results/
echo "Done sixth set of tests "