import cv2 
import numpy as np 
from skimage.measure import ransac 
from skimage.transform import FundamentalMatrixTransform 
from skimage.transform import EssentialMatrixTransform

### feature based SLAM
### opencv orb features 
## funadamental matrix calculator

## Po wyciągnięciu punktów i matchowaniu ich między klatkami chcemy oszacować pose transormation, do tego używamy EssentialMatrix 
# camera focal length - o ile pikseli przesunie się obraz jeśli przekręcimy kamerę o 1 rad 

def extractRt(E): 
    W = np.mat([[0,-1,0],[1,0,0],[0,0,1]], dtype = float)
    U,d,Vt = np.linalg.svd(E)
    assert np.linalg.det(U) >0
    if np.linalg.det(Vt) < 0:
        Vt *= -1.0
    R = np.dot(np.dot(U,W), Vt)
    if np.sum(R.diagonal()) <0: 
        R = np.dot(np.dot(U, W.T), Vt)
    t = U[:,2]
    Rt = np.concatenate([R,t.reshape(3,1)], axis=1)
    return Rt

class ExtractOrbFeatures(object):
    
    def __init__(self,K):
        self.orb = cv2.ORB_create()
        self.BruteMatch = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.last = None 
        self.K = K 
        self.invK = np.linalg.inv(self.K) #odwrócenie macierzy parametrów 
        
    def detectFeature(self,img):  
        f_est_avg = [] 
        
        img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        (thresh, img_bw) = cv2.threshold(img_gray, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        # features extraction
        features = cv2.goodFeaturesToTrack(img_bw,3000,qualityLevel=0.1,minDistance=3)
        KeyPoints = [cv2.KeyPoint(x=feature[0][0],y=feature[0][1], size=20) for feature in features]
        KeyPoints,descriptors = self.orb.compute(img,KeyPoints)
        
        return_matches = []
        if self.last != None: 
            matches = self.BruteMatch.knnMatch(descriptors,self.last[1],k=2)  
            for m,n in matches:
                if m.distance < 0.75*n.distance:
                    keypoint1 = KeyPoints[m.queryIdx].pt
                    keypoint2 = self.last[0][m.trainIdx].pt
                    return_matches.append((keypoint1,keypoint2))
                    
        Rt = None 
        if len(return_matches) > 0: 
            return_matches = np.array(return_matches)
            ### Normalizacja koordynat - znalezienie srodka 
            return_matches[:,:,0] -=  img.shape[0]//2
            return_matches[:,:,1] -=  img.shape[1]//2
            # filtracja matchy 
            model, inliers = ransac((return_matches[:,0], return_matches[:,1]),
                                    #EssentialMatrixTransform,
                                    FundamentalMatrixTransform, 
                                    min_samples = 8, residual_threshold=1, max_trials=100 )
            
            return_matches = return_matches[inliers]
            #print(model.params)
            Rt = extractRt(model.params)
                  
        self.last = [KeyPoints,descriptors]
        return return_matches, Rt           
        
        
        
######## Main file ########## 
W = 1920//2 #dzielenie intigerow w pythonie 
H = 1080//2
F = 160
   
K = np.array(([F,0,W//2],[0,F,H//2],[0,0,1]))  # intrinsic parameters matrix, F - camera focal length
extractor = ExtractOrbFeatures(K) 

def readVideo(img): 
    img = cv2.resize(img,(W,H))
    matches, Rt = extractor.detectFeature(img)
    if Rt is None: 
        return
    print(Rt)
    
    print("%d matches" % (len(matches))) 
    
    for point1, point2 in matches:
        u1,v1 = map(lambda x: int(round(x)),point1) # u, v - koordynaty punktu 
        u2,v2 = map(lambda x: int(round(x)),point2)
        
        u1 += img.shape[0]//2 #denormalizacja koordynat punktów 
        u2 += img.shape[0]//2
        v1 += img.shape[1]//2 #denormalizacja koordynat punktów 
        v2 += img.shape[1]//2
        cv2.circle(img,(u1,v1),color = (0,255,0), radius = 3)
        cv2.line(img,(u1,v1),(u2,v2),color = (255,0,0), thickness = 2)
    
    cv2.imshow("vSlam",img)
    cv2.waitKey(10)


if __name__ == "__main__": 
    cap = cv2.VideoCapture("test.mp4")
    
    while cap.isOpened():
        ret, frame = cap.read() #ret ostrzymuje wartość czy ramki są czytane czy nie
        if ret == True: 
            readVideo(frame)
        else: 
            break