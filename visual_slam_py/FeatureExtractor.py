import cv2 
import numpy as np
from skimage.measure import ransac 
from skimage.transform import FundamentalMatrixTransform 
from skimage.transform import EssentialMatrixTransform
from tools import *

class FeatureExtractor(object): 
    def __init__(self, num_of_features, intrinsicMatrix) -> None:
        self.orb = cv2.ORB_create(num_of_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        
        self.numKP = num_of_features
        self.K = intrinsicMatrix
        self.Kinv = np.linalg.inv(self.K)
        self.cx = intrinsicMatrix[0][2]
        self.cy = intrinsicMatrix[1][2]
        self.f = intrinsicMatrix[0][0]
        
        self.last = None 
        self.prev_KP = None
        self.E = None 
        

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
                    
        return np.array([(keyP.pt[0], keyP.pt[1]) for keyP in keypoints]), descriptors
    
    def BFmatcher(self, keypoints, descriptors): 

        return_matches = []
        if self.last != None: 
            matches = self.matcher.knnMatch(descriptors,self.last[1],2)  
            for m,n in matches:
                if m.distance < 0.75*n.distance:
                    keypoint1 = keypoints[m.queryIdx]
                    keypoint2 = self.last[0][m.trainIdx]
                    return_matches.append((keypoint1,keypoint2))
        
        self.last = [keypoints,descriptors]
        
        print("number of matches pre ransac: ", len(return_matches))
        
        if len(return_matches) > 8: 
            
            return_matches = np.array(return_matches)
            ## Normalizacja koordynat - znalezienie srodka 
            # return_matches[:,:,0] -= self.cx
            # return_matches[:,:,1] -= self.cy
            return_matches[:,0,:]  = normalize(return_matches[:,0,:], self.Kinv)
            return_matches[:,1,:]  = normalize(return_matches[:,1,:], self.Kinv)
    
            #filtracja matchy             
            self.E, inliers = ransac((return_matches[:,0], return_matches[:,1]),
                                    EssentialMatrixTransform,
                                    min_samples = 8, 
                                    residual_threshold=0.02, 
                                    max_trials=100)
            
            return_matches = return_matches[inliers]
            #print("matches = ", return_matches)
            print("number of matches after ransac: ", len(return_matches))
            # print("Fundamental matrix = \n", self.E.params)
        
        return return_matches
        
        
    def getPose(self): 
        
        W = np.mat([[0,-1,0], [1,0,0], [0,0,1]])
        R = np.mat([[1, 0, 0], [0,1,0], [0,0,1]]) 
        t = np.array([0, 0 ,0])
        if self.E != None:
            u, w, vt = np.linalg.svd(self.E.params)
            assert np.linalg.det(u) > 0
            if np.linalg.det(vt) < 0: 
                vt *=-1.0
            
            R = np.dot(np.dot(u,W), vt)
            if np.sum(R.diagonal()) < 0: 
                R = np.dot(np.dot(u,W.T), vt)
            t = u[:,2]
            # print(" macierz rotacji \n", R)
            # print("suma na diagonali R",np.sum(R.diagonal()))
        
        pose = np.concatenate([R, t.reshape(3,1)], axis = 1)
        return pose 
