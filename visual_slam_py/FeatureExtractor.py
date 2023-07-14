import cv2 
import numpy as np
from skimage.measure import ransac 
from skimage.transform import FundamentalMatrixTransform 
from skimage.transform import EssentialMatrixTransform
from tools import *

        
def extractFeatures(img, num_of_features):
    orb = cv2.ORB_create(num_of_features)
    # GY, GX = 10, 2
    # H = img.shape[0]
    # W = img.shape[1]
    # sy, sx = H//GY, W//GX
    # all_keypoints = []
    # for ry in range(0, W, sx): 
    #     for rx in range(0, H, sy): 
    #         img_chunk = img[ry:ry+sy, rx:rx+sx]
    #         keyPoints, descriptors = self.orb.detectAndCompute(img_chunk, None)
    
    corners = cv2.goodFeaturesToTrack(img, num_of_features, qualityLevel= 0.01, minDistance=3)
    keypoints = [cv2.KeyPoint(x = corner[0][0], y = corner[0][1], size = 20) for corner in corners]
    keypoints, descriptors = orb.compute(img, keypoints)
    #keypoints = cv2.KeyPoint_convert(corners, size=20)
    return np.array([(keyP.pt[0], keyP.pt[1]) for keyP in keypoints]), descriptors
    
def matchFrames(Frame1, Frame2): 

    return_matches = []
    firstDesIdxs, secondDesIdxs = [], []; 
    Rt = np.zeros((4,4))
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    
    matches = matcher.knnMatch(Frame1.descriptos, Frame2.descriptos, k = 2)  
    #low's ratio test - basic filtration 
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            firstDesIdxs.append(m.queryIdx)
            secondDesIdxs.append(m.trainIdx)
            
            keypoint1 = Frame1.featurePts[m.queryIdx]
            keypoint2 = Frame2.featurePts[m.trainIdx]
            return_matches.append((keypoint1,keypoint2))
   
    print("number of matches pre ransac: ", len(return_matches))

    
    if len(return_matches) >= 8: 
       
        firstDesIdxs = np.array(firstDesIdxs)
        secondDesIdxs = np.array(secondDesIdxs)
        return_matches = np.array(return_matches)
        #filtracja matchy             
        E, inliers = ransac((return_matches[:,0], return_matches[:,1]),
                                EssentialMatrixTransform,
                                min_samples = 8, 
                                residual_threshold=0.02, 
                                max_trials=100)
        
        return_matches = return_matches[inliers]

        print("number of matches after ransac: ", len(return_matches))

        Rt = getPose(E) 
            
    return firstDesIdxs[inliers], secondDesIdxs[inliers], return_matches, Rt
   
   
def getPose(EssentailMatrix): 
   
    W = np.mat([[0,-1,0], [1,0,0], [0,0,1]])
    R = np.mat([[1, 0, 0], [0,1,0], [0,0,1]]) 
    t = np.array([0, 0 ,0])
    Rt = np.eye(4)
    
    if EssentailMatrix!= None:
        u, w, vt = np.linalg.svd(EssentailMatrix.params)
        assert np.linalg.det(u) > 0
        if np.linalg.det(vt) < 0: 
            vt *=-1.0
        
        R = np.dot(np.dot(u,W), vt)
        if np.sum(R.diagonal()) < 0: 
            R = np.dot(np.dot(u,W.T), vt)
        t = u[:,2] 
        # print(" macierz rotacji \n", R)
        # print("suma na diagonali R",np.sum(R.diagonal()))
        
    Rt[:3, :3] = R 
    Rt[:3,3] = t
    return Rt 


class Frame(): 
    def __init__(self, img, intrinsicMatrix, num_of_features):
                
        self.K = intrinsicMatrix
        self.Kinv = np.linalg.inv(self.K)
        self.cx = intrinsicMatrix[0][2]
        self.cy = intrinsicMatrix[1][2]
        self.f = intrinsicMatrix[0][0] 
        self.pose = np.eye(4)    # homogenous transformation matrix 
        
        featurePoints, self.descriptos = extractFeatures(img, num_of_features)
        self.featurePts = normalize(featurePoints, self.Kinv)
        