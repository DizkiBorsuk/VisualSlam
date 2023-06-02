import cv2 as cv 
import numpy as np



def computeStereoCorrespondance(left_img, right_img, matcher_type): 
    
    sad_window = 6
    num_disparities = sad_window*16
    block_size = 11
    
    
    matcher = cv.StereoBM_create(numDisparities=num_disparities, blockSize= block_size)
    
    disparitie_map = matcher.compute(left_img, right_img).astype(np.float32)/16
     
    return disparitie_map  
