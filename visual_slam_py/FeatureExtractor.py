import cv2 
import numpy as np
from skimage.measure import ransac 
from skimage.transform import FundamentalMatrixTransform 
from skimage.transform import EssentialMatrixTransform

class FeatureExtractor(object): 
    def __init__(self, num_of_features, cx, cy) -> None:
        self.orb = cv2.ORB_create(num_of_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        
        self.numKP = num_of_features
        self.last = None 
        self.prev_KP = None
        self.cx = cx
        self.cy = cy
    
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
        keypoints, descriptors = self.orb.compute(img, keypoints)
        #keypoints = cv2.KeyPoint_convert(corners, size=20)
                    
        return keypoints, descriptors
    
    def BFmatcher(self, keypoints, descriptors): 

        return_matches = []
        if self.last != None: 
            matches = self.matcher.knnMatch(descriptors,self.last[1],2)  
            for m,n in matches:
                if m.distance < 0.75*n.distance:
                    keypoint1 = keypoints[m.queryIdx].pt
                    keypoint2 = self.last[0][m.trainIdx].pt
                    return_matches.append((keypoint1,keypoint2))
        
        self.last = [keypoints,descriptors]
        
        print("number of matches pre ransac: ", len(return_matches))
        
        if len(return_matches) > 10: 
            return_matches = np.array(return_matches)
            
            ## Normalizacja koordynat - znalezienie srodka 
            return_matches[:,:,0] -=  self.cx
            return_matches[:,:,1] -=  self.cy
            
            #filtracja matchy 
            model, inliers = ransac((return_matches[:,0], return_matches[:,1]),
                                    #EssentialMatrixTransform,
                                    FundamentalMatrixTransform, 
                                    min_samples = 8, 
                                    residual_threshold=1, 
                                    max_trials=100)
            
            return_matches = return_matches[inliers]
            print("number of matches after ransac: ", len(return_matches))
        
        return return_matches
        
        
        