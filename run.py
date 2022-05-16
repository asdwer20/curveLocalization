import CurveLocalizer
import numpy as np

if __name__=="__main__":
    
    map = np.asarray([0,0,0,10,20,30,20,10,0,0,0,0,0,10,20,30,20,10,0,0])
    lookAhead = 2;
    mapSampleDist = 1
    windowSize = 3
    
    cl = CurveLocalizer.CurveLocalizer(map, windowSize, mapSampleDist, lookAhead)
    