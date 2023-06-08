import cv2 
import numpy as np 
from DatasetRead import *
from FeatureExtractor import * 



kitti = ImportKittyDataset("07")
P0, K0, _, _ = kitti.getCameraMatrixies()

featuresExtractor = FeatureExtractor(1000) 



def main(img): 
    
    keyPoints,descriptors = featuresExtractor.extractFeatures(img)
    for point in keyPoints: 
        u,v = map(lambda x: int(round(x)), point.pt) # img coordinates of each KeyPoint 
        cv2.circle(img, (u,v),color = (0,0,255), radius=3)
    cv2.imshow("vSlam",img)
    cv2.waitKey(33)


if __name__ == "__main__": 
     
    cap = cv2.VideoCapture("../KITTY_dataset/sequences/07//image_0/00%04d.png", cv2.CAP_IMAGES)
    
    while cap.isOpened():
        ret, frame = cap.read() #ret ostrzymuje wartość czy ramki są czytane czy nie
        if ret == True: 
            main(frame)
        else: 
            break
    cap.release()
    cv2.destroyAllWindows()