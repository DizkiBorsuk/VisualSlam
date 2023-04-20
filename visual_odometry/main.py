import cv2 as cv 
import numpy as np
from DatasetRead import ImportKittyDataset 
from ComputeStereo import computeStereoCorrespondance
import matplotlib.pyplot as plt



kitty_data = ImportKittyDataset('06') 



def main():

    
    gt_poses = kitty_data.getGTposes()
    P_l, K_L, P_r, K_r = kitty_data.getCameraMatrixies() 
    left_images, right_images = kitty_data.readImgs('stereo')
    left_img = cv.cvtColor(left_images[0], cv.COLOR_BGR2GRAY)
    right_img = cv.cvtColor(right_images[0], cv.COLOR_BGR2GRAY)
    
    
    depth_img = computeStereoCorrespondance(left_img, right_img, matcher_type='bm')
    
    plt.figure(figsize=(11,7))
    #plt.imshow()
    plt.imshow(depth_img)
    plt.show()


if __name__ == "__main__": 
    main()