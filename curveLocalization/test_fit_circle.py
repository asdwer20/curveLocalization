import numpy as np
import curveLocalization.CurveLocalizer as CurveLocalizer

if __name__=="__main__":
    testMap = [0, 0.1, -.1, 0,.2,.1,0,28,31,28.5,25,35,31.5,30,30,0,0,-2.5,-2.5,1,0,1.6,3.2,0,31,30,32.5,33,29,28,30,31,27]

    mapSampleDist = 0.01

    # Test Parameters
    vx = 0.085543996532293
    steeringAngle = 0.57437315819934
    time = 0.5
    startIdx = 6
    measurement = np.array([0.1, 0.1, 1.7])
    cl = CurveLocalizer.CurveLocalizer(testMap, mapSampleDist, startIdx)

    # points on a unit circle:
    x = [1.2,0,-1]
    y = [0,2,0]

    cl.loadPts(x,y)
    cl.fitcircle()


