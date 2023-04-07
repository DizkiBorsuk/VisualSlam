import cv2 as cv 
import numpy as np
from DatasetRead import ImportKittyDataset 


def main():
    kitty_data = ImportKittyDataset() 
    kitty_data.getGTposes()

    left_images, right_images = kitty_data.readImgs('stereo')

    cv.imshow('left_camera_img', right_images[1])
    cv.waitKey(0)
    cv.destroyAllWindows()



if __name__ == "__main__": 
    main()