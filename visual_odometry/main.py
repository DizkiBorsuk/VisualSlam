import cv2 as cv 
import numpy as np
from DatasetRead import ImportKittyDataset 


def main():
    kitty_data = ImportKittyDataset() 

    left_images = kitty_data.readImgs('mono')

    cv.imshow('left_camera_img', left_images[1])
    cv.waitKey(0)
    cv.destroyAllWindows()



if __name__ == "__main__": 
    main()