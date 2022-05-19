#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Andrew Goering
# Date: 2022-02-09

import rospy
import serial
import utm
import time
from datetime import datetime

from gps_imu_drivers.msg import gps_data

from std_msgs.msg import Header

if __name__ == '__main__':
	filename = '/home/andrewgoering/Northeastern/EECE5554/eece5554/LAB4/' + str(datetime.now()) + '_gps_log.txt'
	f = open(filename , 'w')
	f.write(filename + '\n')
	f.close()
	rospy.init_node('gps')

	sensor_name = rospy.get_param('~sensor_name','gps')
	rospy.loginfo("sensor_name set to %s" %(sensor_name))

	serial_port = rospy.get_param('~port', '/dev/ttyUSB1')
	serial_baud = rospy.get_param('~baudrate', 4800)
	port = serial.Serial(serial_port, serial_baud, timeout = 10)

	gps_pub = rospy.Publisher(sensor_name, gps_data, queue_size = 5)
	r = rospy.Rate(10)
	try:
		s = 0
		while not rospy.is_shutdown():
			try:
				raw = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
				while not (raw[0] == '$GPGGA') and not rospy.is_shutdown():
					# rospy.loginfo(raw)
					raw = port.readline().decode('utf-8')
					f = open(filename, 'a')
					f.write(str(time.time()) + ',' + str(raw))
					f.close()
					raw = raw.split(',')

				#rospy.loginfo(raw)

				lat = float(raw[2][0:2]) + float(raw[2][2:])/60
				long = float(raw[4][0:3]) + float(raw[4][3:])/60
				alt = float(raw[9])

				if raw[3] == 'S':
					lat = -1*lat
				if raw[5] == 'W':
					long = -1*long

#				rospy.loginfo(str(lat) + 'degrees N')
#				rospy.loginfo(str(long) + 'degrees E')
#				rospy.loginfo(str(alt) + ' m AMS')

				message = gps_data()
				message.header.frame_id = sensor_name
				message.header.seq = s
				s = s + 1
				message.header.stamp = rospy.Time.now()

				message.lat = lat
				message.long = long
				message.alt = alt

				utmloc = utm.from_latlon(lat,long)
				easting = utmloc[0]
				northing = utmloc[1]
				zone = utmloc[2]
				letter = utmloc[3]

				fix = int(raw[6])

#				rospy.loginfo(str(easting) + 'm easting\t' + str(northing) + 'm northing\t' + 'zone ' + str(zone) + str(letter) )

				message.utm_easting = easting
				message.utm_northing = northing
				message.utm_zone = zone
				message.utm_letter = letter

				message.fix = fix
				gps_pub.publish(message)
			except:
                                rospy.logwarn('No valid data') 


	except rospy.ROSInterruptException: pass

