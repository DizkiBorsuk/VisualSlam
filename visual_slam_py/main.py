import cv2 
import numpy as np 
import matplotlib.pyplot as plt 
from DatasetRead import *
from FeatureExtractor import * 
from ComputeStereo import *



kitti = ImportKittyDataset("07") #clas object to get kitti dataset 
P_left, K_left, t_left, P_right, K_right, t_right = kitti.getCameraMatrixies() #get projection and camera matricies 
cx = K_left[0][2]
cy = K_left[1][2]
print("K = ", K_left)
print("cx = ", cx)
print("cy = ", cy)


featuresExtractor = FeatureExtractor(800, cx, cy) # 



def mono_slam(img): 
    
    keyPoints,descriptors = featuresExtractor.extractFeatures(img)
    matches = featuresExtractor.BFmatcher(keyPoints, descriptors)
    
    for matched_point1, matched_point2 in matches: 
        u1,v1 = map(lambda x: int(round(x)), matched_point1) # img coordinates of each KeyPoint 
        u2,v2 = map(lambda x: int(round(x)), matched_point2)
        #denormalizacja 
        u1 += cx
        u2 += cx
        v1 += cy
        v2 += cy
        
        cv2.circle(img, (u1.astype(int),v1.astype(int)),color = (0,0,255), radius=3)
        cv2.line(img,(u1.astype(int),v1.astype(int)),(u2.astype(int),v2.astype(int)),color = (255,0,0), thickness = 2)
        
    cv2.imshow("vSlam",img)
    cv2.waitKey(33)
    
def stereo_slam(frame1, frame2): 
    
    disparityMap = computeStereoCorrespondance(frame1, frame2, matcher_type = 'sgbm') #block_matching or sgbm
    depthMap = computeDepthMap(disparityMap, K_left, t_left, t_right)
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