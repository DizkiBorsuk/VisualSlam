import cv2 
import numpy as np 
import g2o
from Extractor import extractFeatures, matchFrames, Frame


W,H = 1920//2, 1080//2

# intrinsic parameters matrix, F - camera focal length
F = 160
K = np.array(([F,0,W//2],[0,F,H//2],[0,0,1]))  

frames = []
def readVideo(img): 
    img = cv2.resize(img,(W,H))
    frame = Frame(img,K)
    frames.append(frame)
    matches, Rt = matchFrames(frames[-1], frames[-2])
    
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