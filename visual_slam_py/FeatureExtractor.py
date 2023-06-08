import cv2 
import numpy as np


class FeatureExtractor(object): 
    def __init__(self, num_of_features) -> None:
        self.orb = cv2.ORB_create()
        self.matcher = cv2.BFMatcher()
        
        self.numKP = num_of_features
        self.last = None 
    
    def extractFeatures(self,img):
        # GY, GX = 10, 2
        
        # H = img.shape[0]
        # W = img.shape[1]
        # sy, sx = H//GY, W//GX
          
        # all_keypoints = []
        # for ry in range(0, W, sx): 
        #     for rx in range(0, H, sy): 
        #         img_chunk = img[ry:ry+sy, rx:rx+sx]
        #         keyPoints, descriptors = self.orb.detectAndCompute(img_chunk, None)
        corners = cv2.goodFeaturesToTrack(img, self.numKP, qualityLevel= 0.01, minDistance=3)
        keypoints = [cv2.KeyPoint(x = corner[0][0], y = corner[0][1], size = 20) for corner in corners]
        descriptors = self.orb.compute(img, keypoints)
        #keypoints = cv2.KeyPoint_convert(corners, size=20)
        self.last = {'keyPoints': keypoints, 'descriptors': descriptors}
        
        return keypoints, descriptors
    
    def BFmatcher(self, img): 
        
        if self.last is not None: 
            pass  
        
        
        