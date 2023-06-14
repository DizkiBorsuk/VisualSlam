import cv2 
import numpy as np 
from DatasetRead import *
from FeatureExtractor import * 
from ComputeStereo import *



kitti = ImportKittyDataset("07") #clas object to get kitti dataset 
P_left, K_left, t_left, P_right, K_right, t_right = kitti.getCameraMatrixies() #get projection and camera matricies 
print("t_left = ", t_left)
print("t_rigt = ", t_right)
print("K_right ", K_right)
print("K_left = ", K_left)


featuresExtractor = FeatureExtractor(1000) # 



def mono_slam(img): 
    
    keyPoints,descriptors = featuresExtractor.extractFeatures(img)
    for point in keyPoints: 
        u,v = map(lambda x: int(round(x)), point.pt) # img coordinates of each KeyPoint 
        cv2.circle(img, (u,v),color = (0,0,255), radius=3)
    cv2.imshow("vSlam",img)
    cv2.waitKey(33)
    
def stereo_slam(frame1, frame2): 
    
    disparityMap = computeStereoCorrespondance(frame1, frame2, matcher_type = 'block_matching')
    depthMap = computeDepthMap(disparityMap, K_left, t_left, t_right)
    print(depthMap)
    
    
    cv2.imshow("vSlam",disparityMap)
    cv2.waitKey(33)


if __name__ == "__main__": 
     
    cap = cv2.VideoCapture("../KITTY_dataset/sequences/07/image_0/00%04d.png", cv2.CAP_IMAGES)
    cap_right = cv2.VideoCapture("../KITTY_dataset/sequences/07/image_1/00%04d.png", cv2.CAP_IMAGES)
    
    while cap.isOpened():
        ret, frame = cap.read() #ret ostrzymuje wartość czy ramki są czytane czy nie
        ret2, frame_right = cap_right.read() #ret ostrzymuje wartość czy ramki są czytane czy nie
        
        if ret == True: 
            #mono_slam(frame) 
            stereo_slam(frame, frame_right)
            #pass
        else: 
            break
        
    cap.release()
    cv2.destroyAllWindows()