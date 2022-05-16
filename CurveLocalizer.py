from re import S
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
    
    #     mapMat = np.zeros((0,self.window_size))
    #     for i in range(len(self.map)):
    #         idx = list(range(i,i+self.window_size))
    #         tempRow = self.map.take(idx, mode='wrap')
    #         tempRow = np.reshape(tempRow, (1,self.window_size))
    #         mapMat = np.append(mapMat, tempRow, axis=0)
    #         # print(mapMat)
    #     return mapMat
        pass
    

    def findPosition_convolve(self, data):

    #     convResult = convolve1d(map, data, mode='wrap')
        
    #     minPosition = np.argmin(convResult)
        
    #     return minPosition

        pass

    def predict(self, imu_twist, vesc_twist, dt, X_prev):

        ## gather data form imu and vesc
        v_theta = imu_twist[-1]
        v_x = vesc_twist[0]
        v_y = vesc_twist[1]

        ## euler discretization of differential drive model
        X = X_prev + dt*np.array([[v_x], [v_y], [v_theta]])

        return X


    def convolve(self, map_array, window, start, win_size):

        win_sum = np.sum(window)
        b = np.convolve(map_array[int(start - 0.5*win_size)::], np.ones(win_size,dtype=int),'valid')
        curve_num = np.argmin(abs(b-win_sum))

        return curve_num
        
        
    
    def windowSize(self, vesc_twist, v_max, v_min, max_size, min_size):

        # v_x = vesc_twist[0]
        # v_y = vesc_twist[1]

        v = max(vesc_twist[0], vesc_twist[1])

        if v > v_max:
            win_size = max_size
        elif v < v_min:
            win_size = min_size
        else:
            win_size = v/v_max * max_size

        return int(win_size)