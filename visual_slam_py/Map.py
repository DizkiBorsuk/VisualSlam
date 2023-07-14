import numpy as np 


class Point(object): 
    # representation of 3D point in a world 
    def __init__(self, location):
        self.frames = [] 
        self.location = location 
        self.descriptor_indexes = [] #indexes of frames tha point was observed in 
        #
        
    def addObservation(self, frame, idx): 
        self.frames.append(frame)
        self.descriptor_indexes.append(idx)


class Map(object): 
    def __init__(self):
        self.frames = [] 
        self.points = [] 