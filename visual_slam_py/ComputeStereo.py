import cv2 as cv 
import numpy as np



def computeStereoCorrespondance(left_img, right_img, matcher_type): 
    
    sad_window = 6
    num_disparities = sad_window*16
    block_size = 11
    
    if matcher_type == 'block_matching': 
        matcher = cv.StereoBM_create(numDisparities=num_disparities, blockSize=block_size)
        
    elif matcher_type == 'sgbm': 
        matcher = cv.StereoSGBM_create(numDisparities=num_disparities,minDisparity = 0, blockSize=block_size, 
                                    P1 = 16*sad_window**2, P2 = 96*sad_window**2, 
                                    mode = cv.STEREO_SGBM_MODE_SGBM_3WAY)
    
    disparitie_map = matcher.compute(left_img, right_img).astype(np.float32)/16
     
    return disparitie_map  

def computeDepthMap(disparityMap, K_left, t_left, K_right, t_right): 
    
    fx1 = K_left[0][0]

    b = t_right[0] - t_left[0] # b - distance between cameras center points in x direction 
    
    
    depthMap = np.ones(disparityMap.shape)
    depthMap = fx1*b/disparityMap
    
    return depthMap 