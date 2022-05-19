#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import tf

if __name__ == '__main__':

    # Set serial params
    rospy.init_node('imu_node') #
    serial_port = rospy.get_param('~port','/dev/ttyUSB0') #
    serial_baud = rospy.get_param('~baudrate',115200)

    # Open Serial Port
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    #rate = rospy.Rate(40)
    rospy.loginfo("Using IMU on port "+serial_port+" at "+str(serial_baud))
    rospy.logdebug("Initializing sensor")

    # Setup Publisher to publish message type imu_msg to topic imu
    imu_pub = rospy.Publisher('imu', Imu, queue_size=5)
    mag_pub = rospy.Publisher('mag', MagneticField, queue_size=5)

    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing IMU.")

    # Set imu and mag msg variable and define header params
    imu_msg = Imu()
    imu_msg.header.frame_id = "imu"
    imu_msg.header.seq = 0

    mag_msg = MagneticField()
    mag_msg.header.frame_id = "mag"
    mag_msg.header.seq = 0

    try:
        while not rospy.is_shutdown():


            # Read in data from serial port and decode
            line = port.readline().decode('utf-8')
            print(line)

            if line == '':
                print("IMU: No data")
            else:
                if line.startswith('$VNYMR'):
                    imu_msg.header.stamp = rospy.Time.now()
                    mag_msg.header.stamp = rospy.Time.now()

                    # split string data on the comma
                    data = line.split(',')

                    # extract YPR
                    ypr = [float(data[1]),float(data[2]),float(data[3])]

                    # In ZYX Order (quaternion visualization verifies)
                    # quaternion transformation requires RPY in Radians
                    q = tf.transformations.quaternion_from_euler(ypr[2]*pi/180,ypr[1]*pi/180,ypr[0]*pi/180)
                    
                    imu_msg.orientation.x = q[0]
                    imu_msg.orientation.y = q[1]
                    imu_msg.orientation.z = q[2]
                    imu_msg.orientation.w = q[3]

                    # convert from Gauss to Teslas
                    mag = [float(data[4])/10000,float(data[5])/10000,float(data[6])/10000]
                    mag_msg.magnetic_field.x = mag[0] # Teslas
                    mag_msg.magnetic_field.y = mag[1]
                    mag_msg.magnetic_field.z = mag[2]

                    accel = [float(data[7]),float(data[8]),float(data[9])]
                    imu_msg.linear_acceleration.x = accel[0] # m/s
                    imu_msg.linear_acceleration.y = accel[1]
                    imu_msg.linear_acceleration.z = accel[2]

                    # remove the last few characters that look like *6D or *67...
                    gyro = [float(data[10]),float(data[11]),float(data[12][:-5])]
                    imu_msg.angular_velocity.x = gyro[0] # rad/s
                    imu_msg.angular_velocity.y = gyro[1]
                    imu_msg.angular_velocity.z = gyro[2]
                    
                    imu_msg.header.seq += 0
                    mag_msg.header.seq += 0
                    imu_pub.publish(imu_msg)
                    mag_pub.publish(mag_msg)

                    #print(imu_msg)
            #rate.sleep()

    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down imu node...")



