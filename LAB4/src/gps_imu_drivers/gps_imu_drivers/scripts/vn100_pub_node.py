#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Andrew Goering
# Date: 2022-02-09

import rospy
import serial
import tf
import math
import time
from datetime import datetime

from sensor_msgs.msg import MagneticField, Imu
from geometry_msgs.msg import Quaternion, Vector3


from std_msgs.msg import Header

def publish():
    filename = '/home/andrewgoering/Northeastern/EECE5554/eece5554/LAB4/'+str(datetime.now()) + '_vn100_log.txt'
    f = open(filename , 'w')
    f.write(filename + '\n')
    f.close()
    rospy.init_node('vn100')
    
    sensor_name = rospy.get_param('~sensor_name','vn100')
    
    imu_pub = rospy.Publisher(sensor_name + '_imu', Imu, queue_size = 10)
    mag_pub = rospy.Publisher(sensor_name + '_mag', MagneticField, queue_size = 10)
    
    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate', 115200)
    
    port = serial.Serial(serial_port, serial_baud, timeout = 10)
    
    s = 0
    while not rospy.is_shutdown():
        try:
            raw = port.readline().decode('utf-8')
            f = open(filename, 'a')
            f.write(str(time.time()) + ',' + str(raw))
            f.close()
            raw = raw.split(',')
            while not (raw[0][-6:] == '$VNYMR') and not rospy.is_shutdown():
                # rospy.loginfo(raw)
                raw = port.readline().decode('utf-8')
                f = open(filename, 'a')
                f.write(str(time.time()) + ',' + str(raw))
                f.close()
                raw = raw.split(',')
            # rospy.loginfo(raw)

            yaw = math.radians(float(raw[1]))
            pitch = math.radians(float(raw[2]))
            roll = math.radians(float(raw[3]))

            wx = float(raw[10])
            wy = float(raw[11])
            wz = float(raw[12][:-5])

            ax = float(raw[7])
            ay = float(raw[8])
            az = float(raw[9])

            mx = float(raw[4])
            my = float(raw[5])
            mz = float(raw[6])

            covar= [0, 0, 0, 0, 0, 0, 0, 0, 0]

            # rospy.loginfo("\yaw = " + str(yaw))
            # rospy.loginfo("\pitch = " + str(pitch))
            # rospy.loginfo("\roll = " + str(roll))

            q_raw = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes = 'rzyx')
            q = Quaternion()
            q.x = q_raw[0]
            q.y = q_raw[1]
            q.z = q_raw[2]
            q.w = q_raw[3]

            # rospy.loginfo("\nq = " + str(q))

            w = Vector3()
            w.x = wx
            w.y = wy
            w.z = wz

            a = Vector3()
            a.x = ax
            a.y = ay
            a.z = az

            m = Vector3()
            m.x = mx
            m.y = my
            m.z = mz

            imu_msg = Imu()
            imu_msg.header.frame_id = sensor_name
            imu_msg.header.seq = s
            imu_msg.header.stamp = rospy.Time.now()

            imu_msg.orientation = q
            imu_msg.orientation_covariance = covar

            imu_msg.angular_velocity = w
            imu_msg.angular_velocity_covariance = covar

            imu_msg.linear_acceleration = a
            imu_msg.linear_acceleration_covariance = covar


            mag_msg = MagneticField()
            mag_msg.header.frame_id = sensor_name
            mag_msg.header.seq = s
            mag_msg.header.stamp = rospy.Time.now()
            mag_msg.magnetic_field = m
            mag_msg.magnetic_field_covariance = covar

            imu_pub.publish(imu_msg)
            mag_pub.publish(mag_msg)

            s = s + 1
        except:
            rospy.logwarn('invalid data')

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
            
