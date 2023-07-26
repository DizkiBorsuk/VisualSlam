import numpy as np 


class Point(object): 
    # representation of 3D point in a world 
    #each point should be observed in multiple frames (Frame object)
    def __init__(self, location, map):
        self.frames = [] 
        self.location = location 
        self.descriptor_indexes = [] #indexes of frames tha point was observed in 
        self.id = len(map.points)
        map.points.append(self)
        #
        
    def addObservation(self, frame, idx): 
        self.frames.append(frame)
        self.descriptor_indexes.append(idx)


class Map(object): 
    def __init__(self):
        self.frames = [] 
        self.points = [] 
        
    def display(self):
        pass