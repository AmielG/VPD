#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def update_measurements(msg):
    # Intel RealSense t265 corddiate is left handed coordinate.
    global measurements
    measurements = msg


def update_car_commands(msg):
    global commands
    commands = msg


def update_accel(msg):
    # Intel RealSense t265 corddiate is left handed coordinate.
    global accel
    accel = msg


def update_estimate_state(msg):
    # Intel RealSense t265 corddiate is left handed coordinate.
    global estimate_state
    estimate_state = msg


def update_speed(msg):
    global speed
    if not np.isnan(msg.data[SPEED_INDEX]):
        speed = msg.data[SPEED_INDEX]


def update_steering_angle(msg):
    global steering_angle
    steering_angle = msg.data


def get_yaw(quaternion):
    orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return yaw


def state_estimator():
    rospy.init_node('logger2')
    rospy.loginfo("Start logger node.")
    pub = rospy.Publisher('/logger/log', Float32MultiArray, queue_size=1000)
    rospy.Subscriber("/sensors_data_collector/sensors_data", Float32MultiArray, update_speed, queue_size=1)
    rospy.Subscriber("/steering_controller/steering_angle", Float64, update_steering_angle, queue_size=1)
    rospy.Subscriber("/camera/odom/sample", Odometry, update_measurements, queue_size=1)
    rospy.Subscriber("/camera/accel/sample", Imu, update_accel, queue_size=1)
    rospy.Subscriber("car_commands", Odometry, update_car_commands, queue_size=1)
    rospy.Subscriber("/state_estimator/state", Odometry, update_estimate_state, queue_size=1)



    rate = rospy.Rate(50)  # 50hz
    while not rospy.is_shutdown():
        #log_msg.header.stamp = rospy.get_rostim
        yaw = get_yaw(measurements.pose.pose.orientation)
        # arr = [speed, steering_angle, estimate_state.pose.pose.position.x, estimate_state.pose.pose.position.y, measurements.pose.pose.position.x, measurements.pose.pose.position.y, np.rad2deg(estimate_state.pose.pose.orientation.z), np.rad2deg(yaw)]
        throttle = commands.pose.pose.position.x if commands.pose.pose.position.x > 0 else 0
        brake = commands.pose.pose.position.x if commands.pose.pose.position.x < 0 else 0
        arr = [throttle, brake, commands.pose.pose.position.y,
               measurements.pose.pose.position.x, measurements.pose.pose.position.y, measurements.pose.pose.position.z,
               measurements.pose.pose.orientation.x, measurements.pose.pose.orientation.y, measurements.pose.pose.orientation.z, measurements.pose.pose.orientation.w,
               measurements.twist.twist.linear.x, measurements.twist.twist.linear.y, measurements.twist.twist.linear.z,
               measurements.twist.twist.angular.x, measurements.twist.twist.angular.y, measurements.twist.twist.angular.z,
               accel.linear_acceleration.x, accel.linear_acceleration.y, accel.linear_acceleration.z]
        log_msg = Float32MultiArray(data=arr)
        print(log_msg)
        pub.publish(log_msg)
        rate.sleep()


SPEED_INDEX = 0
STEERING_INDEX = 1
measurements = Odometry()
estimate_state = Odometry()
commands = Odometry()
accel = Imu()
steering_angle = 0
speed = 0

if __name__ == '__main__':
    try:
        state_estimator()
    except rospy.ROSInterruptException:
        pass
