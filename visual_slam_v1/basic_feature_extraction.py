import cv2 
import numpy as np 

### feature based SLAM
### opencv orb features 

W = 1920//2 #dzielenie intigerow w pythonie 
H = 1080//2


########### Klasa do extrakcji features bo orb ma kiepski rozkład cech

class ExtractOrbFeatures(object):
    GX = 8
    GY = 6
    
    def __init__(self):
        self.orb = cv2.ORB_create(1000) 
    
    def detectFeature(self,img):  # dzieli obraz na siatkę/mniejsze obrazy i wyciąga z nich cechy
        sy = img.shape[0]//self.GY
        sx = img.shape[1]//self.GX
        keyPoints = [] # wektor zbierający punkty 
        
        for ry in range(0,img.shape[0], sy): #przejście przez wszystkie części obrazu
            for rx in range(0,img.shape[1], sx): 
                smaller_img = img[ry:ry+sy,rx:rx+sx]
                kp= self.orb.detect(smaller_img,None) #detekcja punktów cech w danej części obrazu 
                
                for point in kp: 
                    print(point.pt)
                    point.pt = (point.pt[0] + rx,point.pt[1] + ry) #pt = coordinates of keypoint [x,y]
                    keyPoints.append(point)  #przypisanie punktów w danej klatce i części obrazu do wektora punktów 
                                      
        return keyPoints 
##############

#### Main file 
 
extractor = ExtractOrbFeatures()   
#orb = cv2.ORB_create() 

def readVideo(img): 
    img = cv2.resize(img,(W,H))
    print(img.shape)
    kp = extractor.detectFeature(img)
    
    for point in kp: 
        u,v = map(lambda x: int(round(x)),point.pt)
        cv2.circle(img,(u,v),color = (0,255,0), radius = 3)
        
    cv2.imshow("vSlam",img)
    cv2.waitKey(1)


if __name__ == "__main__": 
    cap = cv2.VideoCapture("test.mp4")
    
    while cap.isOpened():
        ret, frame = cap.read() #ret otrzymuje wartość czy ramki są czytane czy nie
        if ret == True: 
            readVideo(frame)
        else: 
            break
