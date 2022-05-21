from cmath import pi
from turtle import right
import numpy as np
from control.matlab import *
import time

from .CurveLocalizer import CurveLocalizer

#ROS2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu # Might not be needed depending on the output of the gap detector
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32


# CurveLocalizer Class: 
# By taking in the raw laser scan, we compute the upcoming curvature in a look ahead distance away
# (30 degrees either direction from zero), 

NODE_NAME = 'curvloc_node'
MODE = 1 # 1: pure curve detection, #2: everything else
LIDAR_TYPE = 1 # 1: LD06, 2: Hokuyo

LIDAR_TOPIC_NAME = '/scan'
ODOM_TOPIC_NAME = '/odom'
STEERING_TOPIC_NAME = '/steering'

class CurveLocalizerNode(Node):
    def __init__(self):
        # ROS2 communication 
        super().__init__(NODE_NAME)
        self.publiser_ = self.create_publisher(Float32MultiArray, self.publiserCB)
        self.lidar_sub = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.laserScanCB, 10)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC_NAME, self.odomCB, 10)
        self.heading_sub = self.create_subscription(Float32, STEERING_TOPIC_NAME, self.steeringCB, 10)

        # Call position computer
        if MODE == 1:
            self.Ts = 1/100
            self.create_timer(self.Ts, self.run)
        
        # Initialize default parameters
        map_Dir_default = [0,0,0,0,0,0,0]
        map_Dir_default = list(np.array(map_Dir_default, dtype = 'float'))
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mapDir', map_Dir_default),
                ('sampleDist', 0.5),
                ('startIdx', 0),
                ('history_size', 3),
                ('laserFOV', 60)
            ])

        mapDir = self.get_parameter('mapDir').value
        sampleDist = self.get_parameter('sampleDist').value
        startIdx = self.get_parameter('startIdx').value
        history_size = self.get_parameter('history_size').value
        laserFOV = self.get_parameter('laserFOV').value

        # Laser Scan Parameters
        self.laserFOV = laserFOV # FOV in degrees, must be even
        self.newLidarAvailable = False

        # Odometry Data
        self.vx = 0
        self.steering = 0
        self.newVelAvailable = True
        self.newSteerAvailable = True

        self.time_now = time.time()
        self.cl = CurveLocalizer(mapDir, sampleDist, startIdx, history_size, self.time_now)
        self.pu
        

    # Callback Functions
    def laserScanCB(self, msg):
        dtheta = msg.angle_increment # Given in radians
        data = msg.ranges
        angles = list(np.linspace(msg.angle_min, msg.angle_max, int((msg.angle_max-msg.angle_min)/dtheta)+1))
        x,y = self.polar2cart(data,angles)

        # Get Desired Scan Areas
        fov = self.laserFOV*pi/180 #convert fov to radians
        idx_count = int(fov/(dtheta)) #number of indicies to consider on either side

        if LIDAR_TYPE == 1: # LD06
            right90Idx = int((pi/2)/dtheta)
            left90Idx = int((3*pi/2)/dtheta)
            leftx = x[left90Idx:left90Idx+idx_count]
            lefty = y[left90Idx:left90Idx+idx_count]

            rightx = x[right90Idx-idx_count:right90Idx]
            righty = y[right90Idx-idx_count:right90Idx]
        
        if LIDAR_TYPE == 2: # Hokuyo
            right90Idx = int((pi/4)/dtheta)
            left90Idx = int(right90Idx+pi/dtheta)
            leftx = x[left90Idx-idx_count:left90Idx]
            lefty = y[left90Idx-idx_count:left90Idx]       
            
            rightx = x[right90Idx:right90Idx+idx_count]
            righty = y[right90Idx:right90Idx+idx_count]
        
        left_cent, left_curv = self.cl.fitcircle(leftx, lefty)
        right_cent, right_curv = self.cl.fitcircle(rightx, righty)

        if MODE == 1:
            print('left_curv: ', left_curv, 'right_curv: ', right_curv)
        else:
            self.cl.addMeasurement(right_curv) # change this in the future (choose curv)
            self.newLidarAvailable = True

    def odomCB(self, msg):
        self.vx = msg.twist.twist.linear.x
        self.newVelAvailable = True

    def steeringCB(self, msg):
        self.steering = msg.data
        self.newSteerAvailable = True

    def publisherCB(self):
        msg = Float32MultiArray()
        msg.data = [self.currSeq, self.currIdx]

    def polar2cart(self, data, angles):
        # remove nan values
        nan_check = np.vstack((data,angles))
        nan_check = nan_check[:,~np.isnan(nan_check).any(axis=0)]

        data = nan_check[0,:]
        angles = nan_check[1,:]

        x = np.multiply(data,np.cos(angles))
        y = np.multiply(data,np.sin(angles))

        return x,y

    def run(self):
        if self.newLidarAvailable and self.newVelAvailable and self.newSteerAvailable:
            idx = self.cl.computePosition(self.vx, self.steering, time.time(), True)
            msg = Float32MultiArray()
            msg.data = [idx, 0.5]
            self.publiser_.publish(msg)
        if ~self.newLidarAvailable and self.newVelAvailable and self.newSteerAvailable:
            idx = self.cl.computePosition(self.vx, self.steering, time.time(), False)
            msg = Float32MultiArray()
            msg.data = [idx, 0.5]
            self.publiser_.publish(msg)
            

def main(args=None):
    rclpy.init(args=args)
    curvloc_pub = CurveLocalizerNode()
    try:
        rclpy.spin(curvloc_pub)
        curvloc_pub.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        curvloc_pub.get_logger().info(f'{NODE_NAME} shut down successfully')

if __name__ == '__main__':
    main()
