import numpy as np 

# def denormalize(input_points,cx,cy):
#     return int( round(input_points[0] + cx)), int(round(input_points[1]+cy))

def denormalize(input_points, K):
    ret = np.dot(K, np.array([input_points[0], input_points[1], 1.0]))
    # ret /= ret[2]
    return int(round(ret[0])), int(round(ret[1]))

def normalize(input_points, Kinv): 
    return np.dot(Kinv, add_ones(input_points).T).T[:,0:2]

def add_ones(input):
    return np.concatenate([input, np.ones((input.shape[0],1))], axis=1) 
