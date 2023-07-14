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
    
    frame = Frame(img, K, 500) # create Frame object 
    frames.append(frame)
    
    if len(frames) <= 1:
        return
    
    frame1DesIdx, frame2DesIdx, matchedPts, Rt = matchFrames(frames[-1], frames[-2]) # match 
    frames[-1].pose = np.dot(Rt, (frames[-2].pose)) # get real pose of Frame # pose is 4x4 matrix that has rotation and translation in homogenous coordinates
    trajectory.append(frames[-1].pose) 
    
    pointsIn4D = cv2.triangulatePoints(frames[-1].pose[:3],frames[-2].pose[:3], frames[-1].featurePts[frame1DesIdx].T, frames[-2].featurePts[frame2DesIdx].T).T #https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gad3fc9a0c82b08df034234979960b778c
    
    pointsIn4D /= pointsIn4D[:,3:] 
    print("Homogenous points \n", pointsIn4D)
    good4dPts = ((np.abs(pointsIn4D[:,3]) > 0.005) & (pointsIn4D[:,2] > 0))
    pointsIn4D = pointsIn4D[good4dPts] # discard points without enough parallax

    
    frames[-1].pose = np.dot(Rt, (frames[-2].pose).T) # get real pose of Frame 
    
    print("Last frame pose : \n", frames[-1].pose)
    
    
    for point in pointsIn4D: 
        pt = Point(point) 
        pt.addObservation(frames[-1], frame1DesIdx)
        pt.addObservation(frames[-2], frame2DesIdx)
    
    
    for pointsIn1, pointsIn2 in matchedPts: 
        # u1,v1 = map(lambda x: int(round(x)), matched_point1) # img coordinates of each KeyPoint 
        # u2,v2 = map(lambda x: int(round(x)), matched_point2)
        #denormalizacja 
        # u1 += int(round(cx))
        # u2 += int(round(cx))
        # v1 += int(round(cy))
        # v2 += int(round(cy))
        u1,v1 = denormalize(pointsIn1, K)
        u2,v2 = denormalize(pointsIn2, K)
        
        cv2.circle(img, (u1,v1),color = (0,0,255), radius=3)
        cv2.line(img,(u1,v1),(u2,v2),color = (255,0,0), thickness = 2)
    
    
    cv2.imshow("vSlam",img)
    cv.waitKey(33)
    
    
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