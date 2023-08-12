import cv2 
import numpy as np 
import matplotlib.pyplot as plt 
from DatasetRead import *
from FeatureExtractor import * 
from ComputeStereo import *
from tools import *
from Map import * 


dataset = ImportKittyDataset("07") #clas object to get kitti dataset 
P, K, t, P_right, K_right, t_right = dataset.getCameraMatrixies() #get projection and camera matricies 
gtPoses = dataset.getGTposes() 
print(gtPoses.shape)

map = Map() 

frames = []
points = []
trajectory = []


def mono_slam(img): 
    
    frame = Frame(img, K, 500, map) # create Frame object 
    #frames.append(frame)
    
    if frame.id == 0:
        return
    
    frame1 = map.frames[-1] # last frame from map 
    frame2 = map.frames[-2] #pre last frame 
    
    frame1DesIdx, frame2DesIdx, matchedPts, Rt = matchFrames(frame1, frame2) # match 
    frame1.pose = np.dot(Rt, (frame2.pose)) # get real pose of Frame # pose is 4x4 matrix that has rotation and translation in homogenous coordinates
    trajectory.append(frame1.pose) 
    
    pointsIn4D = cv2.triangulatePoints(frame1.pose[:3],frame2.pose[:3], frame1.featurePts[frame1DesIdx].T, frame2.featurePts[frame2DesIdx].T).T #https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gad3fc9a0c82b08df034234979960b778c
    
    pointsIn4D /= pointsIn4D[:,3:] 
    print("Homogenous points \n", pointsIn4D)
    good4dPts = pointsIn4D[:,2] > 0
    # good4dPts = ((np.abs(pointsIn4D[:,3]) > 0.005) & (pointsIn4D[:,2] > 0))
    pointsIn4D = pointsIn4D[good4dPts] # discard points behind camera 
    
    print("Last frame pose : \n", frame1.pose)
    
    
    for i, point in enumerate(pointsIn4D): 
        if not good4dPts[i]:
            continue
        pt = Point(point, map) 
        pt.addObservation(frame1, frame1DesIdx[i])
        pt.addObservation(frame2, frame2DesIdx[i])
    
    
    for pointsIn1, pointsIn2 in matchedPts: 

        u1,v1 = denormalize(pointsIn1, K)
        u2,v2 = denormalize(pointsIn2, K)
        
        cv2.circle(img, (u1,v1),color = (0,0,255), radius=3)
        cv2.line(img,(u1,v1),(u2,v2),color = (255,0,0), thickness = 2)
    
    
    cv2.imshow("vSlam",img)
    cv.waitKey(1)
    
    
def stereo_slam(img1, img2): 
    
    disparityMap = computeStereoCorrespondance(img1, img2, matcher_type = 'sgbm') #block_matching or sgbm
    depthMap = computeDepthMap(disparityMap, K, t, t_right)
    print(disparityMap.size)
    
    cv2.imshow("vSlam",disparityMap)


if __name__ == "__main__": 
     
    cap = cv2.VideoCapture("../KITTY_dataset/sequences/07/image_0/00%04d.png", cv2.CAP_IMAGES)
    cap_right = cv2.VideoCapture("../KITTY_dataset/sequences/07/image_1/00%04d.png", cv2.CAP_IMAGES)
    
    while cap.isOpened():
        ret, img = cap.read() #ret ostrzymuje wartość czy ramki są czytane czy nie
        ret2, img_right = cap_right.read() #ret ostrzymuje wartość czy ramki są czytane czy nie
        
        if ret == True: 
            
            mono_slam(img) 
        
        else: 
            break
   
    cap.release()
    cv2.destroyAllWindows()
    trajectory = np.array(trajectory)
    plt.plot(gtPoses[:,:,3][:,0], gtPoses[:,:,3][:,2])
    plt.plot(trajectory[:,:,3][:,0], trajectory[:,:,3][:,2])
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()