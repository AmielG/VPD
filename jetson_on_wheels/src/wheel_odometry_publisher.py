#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry


class WheelOdometryPublisher:

    def __init__(self):
        rospy.init_node('wheel_odometry_publisher')
        self.SPEED_INDEX = 0
        self.pub = rospy.Publisher('/camera/vehicle_odom_for_t265', Odometry, queue_size=1)
        rospy.Subscriber("/sensors_data_collector/sensors_data", Float32MultiArray, self.publish_speed_to_t265, queue_size=1)

    def run(self):
        rospy.spin()

    def publish_speed_to_t265(self, msg):
        speed = msg.data[self.SPEED_INDEX]
        odom = Odometry()
        odom.twist.twist.linear.x = speed
        self.pub.publish(odom)


if __name__ == '__main__':
    try:
        w = WheelOdometryPublisher()
        w.run()
    except rospy.ROSInterruptException:
        pass
