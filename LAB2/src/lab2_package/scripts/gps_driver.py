#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# I will need a subscriber for analyzing the data
# It has to send data to a csv file that can be imported into matlab

# from rosbag to txt: rostopic echo -b file.bag -p /gps > output.txt

import rospy
import serial
from math import sin, pi
from std_msgs.msg import Float64, String, Int8
from lab2_package.msg import gps_msg
import utm

if __name__ == '__main__':

    rospy.init_node('gps_node') #
    serial_port = rospy.get_param('~port','/dev/ttyACM0') #
    serial_baud = rospy.get_param('~baudrate',115200)
    
    # Open Serial Port
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    rospy.logdebug("Using GPS on port "+serial_port+" at "+str(serial_baud))
    rospy.logdebug("Initializing sensor")
    
    # Setup Publisher to publish message type gps_msg to topic gps
    gps_pub = rospy.Publisher('gps', gps_msg, queue_size=5)

    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing GPS.")
    
    # Set gps_msg variable and define header params
    gps_msg = gps_msg()
    gps_msg.header.frame_id = "gps"
    gps_msg.header.seq=0
   
    try:
        while not rospy.is_shutdown():

            # Read in data from serial port and decode
            line = port.readline().decode('utf-8')
            

            if line == '':
                rospy.logwarn("GPS: No data")
            else:
                if line.startswith('$GNGGA'):
                    #try:
                    # Start Parsing data string
                    gps_msg.header.stamp=rospy.Time.now()
                    #GPGGA,hhmmss.ss,1111.11,a,yyyyy,yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxx*hh
                    words = line.split(',') 

                    # Convert Lat to Degrees from DDMM.MM
                    lat_dm = words[2]
                    lat_d = float(lat_hm[:2])
                    lat_m = float(lat_hm[2:])
                    
                    lad_deg = lat_d + lat_m/60
                    
                    rospy.loginfo(lat_deg)

                    # Convert Long to Degrees from DDDMM.MM
                    lon_dm = words[4]
                    lon_d = float(lon_hm[:3])
                    lon_m = float(lon_hm[3:])
                    
                    lon_deg = lat_d + lat_m/60
                    
                    rospy.loginfo(lon_deg) 

                    alt = float(words[9])
                    fix = words[6]
                    #print(alt)
                    #EASTING, NORTHING, ZONE_NUMBER, ZONE_LETTER
                    utm_list = utm.from_latlon(lat, lon) 

                    # Store parameters in gps_msg
                    gps_msg.latitude = lat
                    gps_msg.longitude = lon
                    gps_msg.altitude = alt
                    gps_msg.utm_easting = utm_list[0]
                    gps_msg.utm_northing = utm_list[1]
                    gps_msg.zone = utm_list[2]
                    gps_msg.letter = utm_list[3]
                    gps_msg.fix = fix
                    print(fix)

                    
                    # Publish gps_msg
                    gps_pub.publish(gps_msg)

                    #except: continue


    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps_node node...")
