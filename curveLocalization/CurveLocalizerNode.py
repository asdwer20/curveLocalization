from cmath import pi
import numpy as np

from CurveLocalizer import CurveLocalizer

#ROS2
import rclpy
from rclpy import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu # Might not be needed depending on the output of the gap detector
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32


# CurveLocalizer Class: 
# By taking in the raw laser scan, we compute the upcoming curvature in a look ahead distance away
# (30 degrees either direction from zero), 

NODE_NAME = 'curvloc_node'

LIDAR_TOPIC_NAME = '/scan'
ODOM_TOPIC_NAME = '/odom'
STEERING_TOPIC_NAME = '/steering'
class CurveLocalizerNode(Node):
    def __init__(self):
        mapDir = self.get_parameter('mapDir').value
        sampleDist = self.get_parameter('sampleDist').value
        startIdx = self.get_parameter('startIdx').value
        history_size = self.get_parameter('hist_size').value
        laserFOV = self.get_parameter('laserFOV').value

        # Laser Scan Parameters
        self.laserFOV = laserFOV # FOV in degrees, must be even
        self.newLidarAvailable = False

        # Odometry Data
        self.vx = 0
        self.steering = 0
        self.newVelAvailable = False
        self.newSteerAvailable = False

        self.cl = CurveLocalizer(mapDir, sampleDist, startIdx, history_size, self.get_clock().now().to_msg())

        # ROS2 communication 
        super().__init__(NODE_NAME)
        #self.publiser_ = self.create_publisher(, self.publiserCB)
        self.lidar_sub = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.laserScanCB)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC_NAME, self.odomCB)
        self.heading_sub = self.create_subscription(Float32, STEERING_TOPIC_NAME, self.steeringCB)

        # Call position computer
        self.Ts = 1/100
        self.create_timer(self.Ts, self.run)

    # Callback Functions
    def laserScanCB(self, msg):
        dtheta = msg.angle_increment # Given in radians
        data = msg.ranges
        angles = list(range(msg.angle_min, len(data), msg.angle_max))
        x,y = self.polar2cart(data,angles)

        fov = self.laserFOV*pi/180 #convert fov to radians
        idx_count = int(fov/(dtheta*2)) #number of indicies to consider on either side
        x = np.append(x[:idx_count], x[-idx_count:])
        y = np.append(y[:idx_count], y[-idx_count:])

        center,curvature = self.cl.fitcircle(x,y)
        self.cl.addMeasurement(curvature)
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
        nan_check = np.sum(np.vstack((data,angles)), axis=0)
        nan_check = nan_check[:,~np.isnan(nan_check).any(axis=0)]
        data = nan_check[0,:]
        angles = nan_check[1,:]

        x = data*np.cos(angles)
        y = data*np.sin(angles)
        return x,y

    def run(self):
        if self.newLidarAvailable and self.newVelAvailable and self.newSteerAvailable:
            self.cl.computePosition(self.vx, self.steering, self.get_clock().now().to_msg(), True)
        if self.newVelAvailable and self.newSteerAvailable:
            self.cl.computePosition(self.vx, self.steering, self.get_clock().now().to_msg(), False)


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