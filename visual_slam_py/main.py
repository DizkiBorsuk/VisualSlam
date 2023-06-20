import cv2 
import numpy as np 
import matplotlib.pyplot as plt 
from DatasetRead import *
from FeatureExtractor import * 
from ComputeStereo import *
from tools import *



kitti = ImportKittyDataset("07") #clas object to get kitti dataset 
P, K, t, P_right, K_right, t_right = kitti.getCameraMatrixies() #get projection and camera matricies 
cx = K[0][2]
cy = K[1][2]
f = K[0][0]
print("K = ", K)
print("cx = ", cx)
print("cy = ", cy)


featuresExtractor = FeatureExtractor(1000, K) # 



def mono_slam(img): 
    
    keyPoints,descriptors = featuresExtractor.extractFeatures(img)
    matches = featuresExtractor.BFmatcher(keyPoints, descriptors)
    
    for matched_point1, matched_point2 in matches: 
        # u1,v1 = map(lambda x: int(round(x)), matched_point1) # img coordinates of each KeyPoint 
        # u2,v2 = map(lambda x: int(round(x)), matched_point2)
        # #denormalizacja 
        # u1 += cx
        # u2 += cx
        # v1 += cy
        # v2 += cy
        u1,v1 = denormalize(matched_point1, K)
        u2,v2 = denormalize(matched_point2, K)
        
        cv2.circle(img, (u1,v1),color = (0,0,255), radius=3)
        cv2.line(img,(u1,v1),(u2,v2),color = (255,0,0), thickness = 2)
        
    cv2.imshow("vSlam",img)
    cv2.waitKey(33)
    
def stereo_slam(frame1, frame2): 
    
    disparityMap = computeStereoCorrespondance(frame1, frame2, matcher_type = 'sgbm') #block_matching or sgbm
    depthMap = computeDepthMap(disparityMap, K, t, t_right)
    print(disparityMap.size)
    
    cv2.imshow("vSlam",disparityMap)
    cv2.waitKey(33)


if __name__ == "__main__": 
     
    cap = cv2.VideoCapture("../KITTY_dataset/sequences/07/image_0/00%04d.png", cv2.CAP_IMAGES)
    cap_right = cv2.VideoCapture("../KITTY_dataset/sequences/07/image_1/00%04d.png", cv2.CAP_IMAGES)
    
    while cap.isOpened():
        ret, frame = cap.read() #ret ostrzymuje wartość czy ramki są czytane czy nie
        ret2, frame_right = cap_right.read() #ret ostrzymuje wartość czy ramki są czytane czy nie
        
        if ret == True: 
            mono_slam(frame) 
            #stereo_slam(frame, frame_right)

        else: 
            break
        
    cap.release()
    cv2.destroyAllWindows()