import numpy as np 
import pandas as pd
import cv2 as cv 
import os



class ImportKittyDataset(): 
    def __init__(self, img_sequance = '00'):
        
        self.sequance_dir = '../KITTY_dataset/sequences/{}/'.format(img_sequance) # directory of images
        self.ground_truth_poses_dir = '../KITTY_dataset/ground_truth_poses/{}.txt'.format(img_sequance) 
        
        self.L_camera_imgs_dir = sorted(os.listdir(self.sequance_dir + 'image_0'))
        self.R_camera_imgs_dir = sorted(os.listdir(self.sequance_dir + 'image_1'))
        self.imgs_num = len(self.L_camera_imgs_dir)
            
        calib = pd.read_csv('../KITTY_dataset/sequences/{}/calib.txt'.format(img_sequance), delimiter=' ',header=None, index_col=0)
        self.P0 = np.array(calib.loc['P0:']).reshape((3,4))
        self.P1 = np.array(calib.loc['P1:']).reshape((3,4)) # P0 and P1 are for grayscale images 
        self.P2 = np.array(calib.loc['P2:']).reshape((3,4)) # P2 and P3 are for RGB images
        self.P3 = np.array(calib.loc['P3:']).reshape((3,4))
        self.Tr = np.array(calib.loc['Tr:']).reshape((3,4))
        
        self.times = np.array(pd.read_csv(self.sequance_dir + 'times.txt', delimiter=' ', header=None))
        
        print('number of frames', self.imgs_num)
        print('Left camera matrix = \n', self.P0)
        print('Right camera matrix = \n', self.P1)
        
        
    def getGTposes(self): 
        ground_truth_poses = pd.read_csv(self.ground_truth_poses_dir, delimiter=' ', header=None) #read ground truth poses
        gt_poses =np.zeros((len(ground_truth_poses), 3, 4))
        
        for npose in range(len(ground_truth_poses)): 
            gt_poses[npose] = (np.array(ground_truth_poses.iloc[npose]).reshape((3,4)))
        
        print('poses shape', gt_poses.shape)
        return gt_poses
        
        
    def readImgs(self, camera_type):
        
        left_images = []
        right_images = [] 
        
        if camera_type == 'mono':
            
            for i,img_name in enumerate(self.L_camera_imgs_dir): 
                left_images.append(cv.imread(self.sequance_dir +'image_0/' + img_name))
            
            self.imheight = left_images[0].shape[0]
            self.imwidth = left_images[0].shape[1]
                
            return left_images
                
        elif camera_type == 'stereo':
 
            for j,img_name in enumerate(self.L_camera_imgs_dir): 
                left_images.append(cv.imread(self.sequance_dir +'image_0/' + img_name))
                
            for k,img_name in enumerate(self.R_camera_imgs_dir): 
                right_images.append(cv.imread(self.sequance_dir +'image_1/' + img_name))
            
            self.imheight = left_images[0].shape[0]
            self.imwidth = left_images[0].shape[1]
            
            return left_images, right_images
            
        else: 
            print('wrong camera format')
            return 1 
      
      
        