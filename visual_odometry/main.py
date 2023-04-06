import cv2 as cv 
import numpy as np
from DatasetRead import Dataset_Handler 


def main():
    kitty_data = Dataset_Handler() 
   
    kitty_data.readImgs('mono')

    cv.imshow('left_camera_img', kitty_data.left_images[4000])
    cv.waitKey(0)
    cv.destroyAllWindows()



if __name__ == "__main__": 
    main()