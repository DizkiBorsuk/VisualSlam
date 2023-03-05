import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import os 
import pandas as pd 
import datetime 





class VisualOdometry(): 
    def __init__(self, groundTruthPoses, K, P):
        self.groundTruth = groundTruthPoses
        self.K = K # intrinsic parameters matrix 
        self.P = P # projection matrix 
        self.orb = cv.ORB_create() 


def main(): 
    
    ## read from kitty dataset 
    ground_truth_poses = pd.read_csv('../KITTY_DATASET/ground_truth_poses/00.txt', delimiter=' ', header=None)
    
    gt_poses = np.zeros((len(ground_truth_poses),3,4))
    for npose in range(len(ground_truth_poses)): 
        gt_poses[npose] = (np.array(ground_truth_poses.iloc[npose]).reshape((3,4)))
        
    print(gt_poses.shape)

    calib_data = pd.read_csv('../KITTY_DATASET/sequences/00/calib.txt', delimiter=' ',header=None, index_col=0)
    



if __name__ == "__main__":
    main()