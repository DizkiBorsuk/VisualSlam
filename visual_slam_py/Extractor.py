import cv2 
import numpy as np 
from skimage.measure import ransac 
from skimage.transform import FundamentalMatrixTransform 
from skimage.transform import EssentialMatrixTransform


def extractFeatures(img): 
    ## initialization and preprocessing 
    orb = cv2.ORB_create() ## Initilize detector 
    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) # Change img to monochromatic 
    (thresh, img_bw) = cv2.threshold(img_gray, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU) # change to binary with otsu treshold
    ## features detection
    features = cv2.goodFeaturesToTrack(img_bw,3000,qualityLevel=0.1,minDistance=3) # detect features with Shi-tomasi detector 
    keyPoints = [cv2.KeyPoint(x=feature[0][0],y=feature[0][1], size=20) for feature in features] #zbiera informacje o punkcie w którym jest cecha
    
    keyPoints,descriptors = orb.compute(img,keyPoints) # create description on features 
    print('keypoints', len(keyPoints))
    
    return np.array([(keypoint.pt[0], keypoint.pt[1]) for keypoint in keyPoints]), descriptors


def matchFrames(frame1, frame2): 
    BruteMatch = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = BruteMatch.knnMatch(frame1.descriptor, frame2.descriptor,k=2) 

    #print(matches)
    #print(len(matches))
    
    return_matches = []
    
    for m,n in matches:
        if m.distance < 0.75*n.distance:        
            point1 = frame1.keypoints[m.queryIdx]        
            point2 = frame2.keypoints[m.trainIdx]      
              
            return_matches.append((point1,point2))     
            return_matches = np.array(return_matches)           
                    
 
            model, inliers = ransac((return_matches[:,0], return_matches[:,1]), # run ransac to get only proper associations (inliers)
                                    #EssentialMatrixTransform,
                                    FundamentalMatrixTransform, 
                                    min_samples = 8, residual_threshold=1, max_trials=100 ) 
            
            return_matches = return_matches[inliers] # from associated between two frames points get the corect ones
            #print(model.params)
            Rt = extractFeatures(model.params)
                  

        return return_matches, Rt    
    
class Frame(object): 
    def __init__(self,img, K):
        self.K = np.array(K) 
        self.Kinv = np.linalg.inv(self.K)
        #self.pose = np.array(pose)
        
        self.H, self.W = img.shape[0:2] #get image shape and width 
        self.keypoints, self.descriptors = extractFeatures(img)
        