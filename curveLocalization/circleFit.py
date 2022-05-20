from scipy import optimize
import numpy as np

class CircleFit:
    def __init__(self, debug):
        self.printDebug = debug

    def calcRadius(self, xc, yc):
        return np.sqrt((self.x-xc)**2 + (self.y-yc)**2)

    def f_2(self, center):
        Ri = self.calcRadius(*center)
        return Ri-np.mean(Ri)

    def fitcircle(self):
        x_m = np.mean(self.x)
        y_m = np.mean(self.y)

        center_est = x_m, y_m
        center, ier = optimize.leastsq(self.f_2, center_est)

        radius = self.calcRadius(*center)

        if self.printDebug:
            print('center: ', center, 'radius: ', radius)

        return center, radius