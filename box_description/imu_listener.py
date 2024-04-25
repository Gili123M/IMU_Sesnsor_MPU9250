#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from math import cos, sin

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

def to_quaternion(roll, pitch, yaw):
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return Quaternion(w, x, y, z)

# Initialize roll, pitch, and yaw
roll, pitch, yaw = 0.0, 0.0, 0.0

def imu_callback(data):
    global roll, pitch, yaw
    
    # Extract gyroscope data from Imu message
    gx = data.angular_velocity.x
    gy = data.angular_velocity.y
    gz = data.angular_velocity.z

    # Integrate gyroscope data using Euler integration
    dt = 1.0 / rospy.get_param('~imu_frequency', 100)  # Assuming imu_frequency parameter is set in the launch file
    roll += gx * dt
    pitch += gy * dt
    yaw += gz * dt

    # Convert Euler angles to quaternion
    orientation_quaternion = to_quaternion(roll, pitch, yaw)

    # Update the Imu message with the orientation quaternion
    data.orientation.x = orientation_quaternion.x
    data.orientation.y = orientation_quaternion.y
    data.orientation.z = orientation_quaternion.z
    data.orientation.w = orientation_quaternion.w

    # Publish the modified Imu message
    imu_publisher.publish(data)

def imu_listener():
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber("imu_data1", Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    # Initialize the publisher with the same topic as the subscriber
    imu_publisher = rospy.Publisher('imu_data1_q', Imu, queue_size=10)

    imu_listener()

