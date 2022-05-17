from re import S
from tracemalloc import start
import numpy as np
import os,pickle
from scipy.ndimage import convolve1d

class CurveLocalizer:
    def __init__(self, mapDir, sampleDist, startIdx):
        # if(os.path.isdir(mapDir)):
        #     with open(mapDir,'rb') as handle:
        #         self.map = pickle.load(handle)
        # else:
        self.map = mapDir       # 1D map array
        
        # create a 2d matrix using the map vector:
        # 2d map matrix= [[0,1,2],
        #                 [1,2,3],
        #                 [2,3,4],
        #                 ....]
        self.sampling_dist = sampleDist
        self.Xprev = np.zeros((3,1))
        self.prevTime = 0
        self.currIdx = startIdx

        self.v_max = 5 
        self.v_min = 2
        self.max_size = 6
        self.min_size = 4

    # def predict(self, imu_twist, vesc_twist, dt, X_prev):
    def predict(self, vx, steeringangle, dt, X_prev):
        l = 0.33
        lr = 0.1
        beta = np.arctan2(lr*(steeringangle),l)
        xdot = vx*np.cos(beta)
        ydot = vx*np.sin(beta)
        thetadot = vx*np.tan(steeringangle)*np.cos(beta)/l

        X = X_prev + dt*np.array([[xdot], [ydot], [thetadot]])

        travelDist = np.linalg.norm(X[:2]-X_prev[:2])

        deltaIdx = int(travelDist/self.sampling_dist)

        self.Xprev = np.copy(X)

        print(travelDist)
        print(deltaIdx)

        return deltaIdx


    # def convolve1d(self, map_array, window, start, win_size):

    #     win_sum = np.sum(window)
        
    #     temp = convolve1d(map_array[int(start - 0.5*win_size):int(start + 0.5*win_size)+1], np.flip(window))
    #     #b = np.convolve(map_array[int(start - 0.5*win_size):int(start + 0.5*win_size)+1], np.ones(self.measurement_len,dtype=int),'valid')
    #     print(temp)
    #     # marker_num = np.argmin((b-temp))
    #     # print()
    #     return int(marker_num+start-0.5*win_size)

    def convolve(self, map_array, window, start, win_size):
        mLen = len(window)
        submap = map_array[int(start - 0.5*win_size):int(start + 0.5*win_size)+1]
        min_sum = 10000
        min_index = 0
        for i in range(len(submap)-len(window)+1):
            curr_sum = np.sum(abs(submap[i:i+len(window)]-window))
            if curr_sum<min_sum:
                min_sum = curr_sum
                min_index = int(start - 0.5*win_size)+i+mLen-1

        return min_index
        
    
    def windowSize(self, vx, v_max, v_min, max_size, min_size):
        if vx > v_max:
            win_size = max_size
        elif vx < v_min:
            win_size = min_size
        else:
            win_size = vx/v_max * max_size

        return int(win_size)

    def computePosition(self, measurements, vx, steeringangle, currTime):
        dt = currTime - self.prevTime
        deltaIdx = self.predict(vx, steeringangle, dt, self.Xprev)
        nextIdx = self.currIdx + deltaIdx
        
        window_size = self.windowSize(vx, self.v_max, self.v_min, self.max_size, self.min_size)
        curve_num = self.convolve(self.map, measurements, nextIdx, window_size)

        return curve_num
