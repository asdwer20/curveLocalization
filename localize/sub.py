#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def imu_callback(msg):
    rospy.loginfo('IMU DATA:', msg.linear_acceleration.x)

def laser_callback(msg):
    rospy.loginfo('LASER DATA:', msg.ranges)
    
def odom_callback(msg):
    rospy.loginfo('ODOM DATA:', msg.twist)


def data_subs():
    rospy.init_node('data_subs', anonymous=True)
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    rospy.Subscriber('Laser', LaserScan, laser_callback)
    rospy.Subscriber('Odom', Odometry, odom_callback)

    rospy.spin()


if __name__ == '__main__':
    data_subs()
    # try:
    #     data_subs()
    # except rospy.ROSInterruptException:
    #     pass