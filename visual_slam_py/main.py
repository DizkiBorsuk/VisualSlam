import cv2 
import numpy as np 
from DatasetRead import *
from FeatureExtractor import * 
from ComputeStereo import *



kitti = ImportKittyDataset("07") #clas object to get kitti dataset 
P0, _, P1, _ = kitti.getCameraMatrixies() #get projection and camera matricies 

K0, r0, t0 = cv2.decomposeProjectionMatrix(P0)
t0 = (t0 / t0[3])[:3]
K1, r1, t1 = cv2.decomposeProjectionMatrix(P1)
t1 = (t1 / t1[3])[:3]


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
    
    cv2.imshow("vSlam",disparityMap)
    cv2.waitKey(33)


if __name__ == "__main__": 
     
    cap = cv2.VideoCapture("../KITTY_dataset/sequences/07/image_0/00%04d.png", cv2.CAP_IMAGES)
    cap_right = cv2.VideoCapture("../KITTY_dataset/sequences/07/image_1/00%04d.png", cv2.CAP_IMAGES)
    
    while cap.isOpened():
        ret, frame = cap.read() #ret ostrzymuje wartość czy ramki są czytane czy nie
        ret2, frame_right = cap.read() #ret ostrzymuje wartość czy ramki są czytane czy nie
        
        if ret == True: 
            #mono_slam(frame) 
            stereo_slam(frame, frame_right)
            
        else: 
            break
        
    cap.release()
    cv2.destroyAllWindows()