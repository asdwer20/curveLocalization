import curveLocalization.CurveLocalizerNode as CurveLocalizerNode
import numpy as np

if __name__=="__main__":
    
    # test_dict = {
    #         ‘0’: [0,0],
    #         ‘1’: [0,0.1],
    #         ‘2’: [0,-0.1],
    #         ‘3’: [0,0],
    #         ‘4’:[ 0,0.2],
    #         ‘5’: [0, 0.1],
    #         ‘6’: [0,0],
    #         ‘7’: [0,28],
    #         ‘8’: [0.5,31],
    #         ‘9’: [0.5,28.5],
    #         ‘10’: [0.5,25],
    #         ‘11’: [0.5,35],
    #         ‘12’: [0.5,31.5],
    #         ‘13’: [0.5,30],
    #         ‘14’: [0.5,30],
    #         ‘15’: [0.5,0],
    #         ‘16’: [0,0],
    #         ‘17’: [0,-2.5],
    #         ‘18’: [0,-2.5],
    #         ‘19’: [0,1],
    #         ‘20’: [0,0],
    #         ‘21’: [0,1.6],
    #         ‘22’: [0,3.2],
    #         ‘23’: [0,0],
    #         ‘24’: [0,31],
    #         ‘25’: [-0.5,30],
    #         ‘26’: [-0.5,32.5],
    #         ‘27’: [-0.5,33],
    #         ‘28’: [-0.5,29],
    #         ‘31’: [-0.5,28],
    #         ‘32’: [-0.5,30],
    #         ‘33’: [-0.5,31],
    #         ‘34’: [-0.5,27]
    # }

    testMap = [0, 0.1, -.1, 0,.2,.1,0,28,31,28.5,25,35,31.5,30,30,0,0,-2.5,-2.5,1,0,1.6,3.2,0,31,30,32.5,33,29,28,30,31,27]

    mapSampleDist = 0.01

    # Test Parameters
    vx = 0.085543996532293
    steeringAngle = 0.57437315819934
    time = 0.5
    startIdx = 6
    measurement = np.array([0.1, 0.1, 1.7])
    cl = CurveLocalizerNode.CurveLocalizer(testMap, mapSampleDist, startIdx)

   
    curve_num = cl.computePosition(measurement, vx, steeringAngle, time)

print(curve_num)
    