import numpy as np 
import pandas as pd
import cv2 as cv 
import os



class ImportKittyDataset(): 
    def __init__(self, img_sequance = '00'):
        
        self.img_sequance = img_sequance
        self.img_dir = '../KITTY_dataset/sequences/{}/'.format(img_sequance) # directory of images
        self.gt_dir = '../KITTY_dataset/ground_truth_poses/{}.txt'.format(img_sequance) # directory with ground truth poses for n sequance
        
        ground_truth_poses = pd.read_csv(self.gt_dir, delimiter=' ', header=None) #read ground truth poses
        self.gt =np.zeros((len(ground_truth_poses), 3, 4))
        
        for i in ground_truth_poses: 
            self.gt[i] = np.array(ground_truth_poses.iloc[i]).reshape((3,4))
            
        self.times = np.array(pd.read_csv('../KITTY_dataset/sequences/{}/times.txt'.format(img_sequance), delimiter=' ', header=None))
        calib = pd.read_csv('../KITTY_dataset/sequences/{}/calib.txt'.format(self.img_sequance), delimiter=' ',header=None, index_col=0)
        self.P0 = np.array(calib.loc['P0:']).reshape((3,4))
        self.P1 = np.array(calib.loc['P1:']).reshape((3,4))
        self.P2 = np.array(calib.loc['P2:']).reshape((3,4))
        self.P3 = np.array(calib.loc['P3:']).reshape((3,4))
        
    def readImgs(self, camera_type):
        
        if camera_type == 'mono':
            self.L_images_dir = os.listdir(self.img_dir + 'image_0')
            self.left_images = [] 
            
            for i,img_name in enumerate(self.L_images_dir): 
                self.left_images.append(cv.imread(self.img_dir +'image_0/' + img_name))
                
        elif camera_type == 'stereo':
            self.L_images_dir =  os.listdir(self.img_dir + 'image_0')
            self.R_images_dir = os.listdir(self.img_dir + 'image_1')
            
            self.left_images = [] 
            self.right_images = [] 
            
            for i,left_img_name,j, rigth_img_name in enumerate(self.L_images_dir, self.R_images_dir): 
                self.left_images.append(cv.imread(self.L_images_dir + left_img_name))
                self.right_images.append(cv.imread(self.R_images_dir + rigth_img_name))
            
        else: 
            print('wrong camera format')
            return 1 
        
        self.imheight = self.left_images[0].shape[0]
        self.imwidth = self.left_images[0].shape[1]
        

    def readLidar(self): 
        pass           
      
        