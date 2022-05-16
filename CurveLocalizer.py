import numpy as np
import os,pickle
from scipy.ndimage import convolve1d

class CurveLocalizer:
    def __init__(self, mapDir, windowSize, sampleDist, lookAhead):
        # if(os.path.isdir(mapDir)):
        #     with open(mapDir,'rb') as handle:
        #         self.map = pickle.load(handle)
        # else:
        self.map = mapDir
            
        # create a 2d matrix using the map vector:
        # i.e. map = [0,1,2,3,4]
        # 2d map matrix= [[0,1,2],
        #                 [1,2,3],
        #                 [2,3,4],
        #                 ....]
        self.window_size = windowSize
        self.look_ahead_distance = lookAhead
        self.sampling_dist = sampleDist
        
        self.mapMatrix = self.createMapMatrix()
        
    def createMapMatrix(self):
        mapMat = np.zeros((0,self.window_size))
        for i in range(len(self.map)):
            idx = list(range(i,i+self.window_size))
            tempRow = self.map.take(idx, mode='wrap')
            tempRow = np.reshape(tempRow, (1,self.window_size))
            mapMat = np.append(mapMat, tempRow, axis=0)
            print(mapMat)
        return mapMat
    
    def findPosition_convolve(self, data):
        convResult = convolve1d(map, data, mode='wrap')
        
        minPosition = np.argmin(convResult)
        
        return minPosition
        
        
    
            
