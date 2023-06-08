import cv2 
import numpy as np 
from DatasetRead import * 


kitti = ImportKittyDataset("07")
P0, K0, _, _ = kitti.getCameraMatrixies()


def main(img): 
    

    cv2.imshow("vSlam",img)
    cv2.waitKey(10)


if __name__ == "__main__": 
    
    cap = cv2.VideoCapture("../KITTY_dataset/sequences/07//image_0/00%04d.png", cv2.CAP_IMAGES)
    
    while cap.isOpened():
        ret, frame = cap.read() #ret ostrzymuje wartość czy ramki są czytane czy nie
        if ret == True: 
            main(frame)
        else: 
            break
    cap.release()
    cv2.destroyAllWindows()