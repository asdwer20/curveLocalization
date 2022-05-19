from re import S
from tracemalloc import start
import numpy as np
import os,pickle
from scipy.ndimage import convolve1d
from scipy import optimize

class CurveLocalizer:
    def __init__(self, mapDir, sampleDist, startIdx):
        self.map = mapDir
        
        self.sampling_dist = sampleDist
        self.Xprev = np.zeros((3,1))
        self.prevTime = 0
        self.currIdx = startIdx

        self.v_max = 5 
        self.v_min = 2
        self.max_size = 6
        self.min_size = 4
        
        # X,Y coordinates of target scan 
        self.x = 0 
        self.y = 0

    def loadPts(self, pts):
        self.x = pts[0]
        self.y = pts[1]

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

    # Functions to fit circle to set of points
    def calcRadius(self, xc, yc):
        return sqrt((self.x-xc)**2 + (self.y-yc)**2)

    def f_2(self, center):
        Ri = calcRadius(*center)
        return Ri-Ri.mean()

    def fitcircle(self):
        x_m = mean(self.x)
        y_m = mean(self.y)

        center_est = x_m, y_m
        center, ier = optimize.leastsq(f_2, center_est)

        radius = calcRadius(*center)

        print('center: ', center, 'radius: ', radius)

        return center, radius

    def computePosition(self, measurements, vx, steeringangle, currTime):
        dt = currTime - self.prevTime
        deltaIdx = self.predict(vx, steeringangle, dt, self.Xprev)
        nextIdx = self.currIdx + deltaIdx
        
        window_size = self.windowSize(vx, self.v_max, self.v_min, self.max_size, self.min_size)
        curve_num = self.convolve(self.map, measurements, nextIdx, window_size)

        return curve_num

