#!/usr/bin/env python3
import rospy
import serial
import socket
import json
from pymavlink import mavutil
import numpy as np
import math as mt

PX4 = mavutil.mavlink_connection("/dev/ttyACM0",115200)
PX4.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (PX4.target_system, PX4.target_system))

This_Drone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
This_Drone.bind(('192.168.3.184',8100))
This_Drone.settimeout(0.1)

Other_Drone = ('192.168.3.183',8100)

Results_File_REL = open(r"/home/ubuntu/ali_ws/logs/Results_File_REL.txt","w+")
Results_File_DRN1 = open(r"/home/ubuntu/ali_ws/logs/Results_File_DRN1.txt","w+")
Results_File_DRN2 = open(r"/home/ubuntu/ali_ws/logs/Results_File_DRN2.txt","w+")
Results_File_DRN1_Local = open(r"/home/ubuntu/ali_ws/logs/Results_File_DRN1_Local.txt","w+")

class GPS_data:
	lat=0
	lon=0
	alt=0
	rel_alt=0
	vx=0
	vy=0
	vz=0
	hdg=0

GPS1 = GPS_data()
GPS2 = GPS_data()

class NED_vector:
	x=0.0
	y=0.0
	z=0.0

NED_rel_pos = NED_vector()
NED_rel_vel = NED_vector()

GPS1_Local_NED = NED_vector()

jsn_GPS_This_Drone = {}
jsn_GPS_Other_Drone = {}

def clean_shutdown():
        Results_File_REL.close()
	Results_File_DRN1.close()
        Results_File_DRN2.close()
        Results_File_DRN1_Local.close()
	print("The connection is terminated!")

def GPS2NED_conversion(inp_GPS1,inp_GPS2):
	#(inp_GPS1 is the GPS data of this drone --- inp_GPS2 is the GPS data of the other drone)
	# The output of this function is the relative position of the other drone to this drone in NED frame
	global NED_rel_pos, NED_rel_vel

	delta_lat = (float(inp_GPS2.lat - inp_GPS1.lat))/1.0e7
	delta_lon = (float(inp_GPS2.lon - inp_GPS1.lon))/1.0e7

	lat0 = (float(inp_GPS1.lat))/1.0e7
	lon0 = (float(inp_GPS1.lon))/1.0e7

	# source: https://en.wikipedia.org/wiki/Geographic_coordinate_system
	NED_rel_pos.x = delta_lat*(111132.92-559.82*mt.cos(2.0*lat0*mt.pi/180.0) + 
			1.175*mt.cos(4.0*lat0*mt.pi/180.0) - 0.0023*mt.cos(6.0*lat0*mt.pi/180.0))
	NED_rel_pos.y = delta_lon*(111412.84*mt.cos(lat0*mt.pi/180)-93.5*mt.cos(3.0*lat0*mt.pi/180.0) + 
			0.118*mt.cos(5.0*lat0*mt.pi/180))
	NED_rel_pos.z = - float(inp_GPS2.rel_alt - inp_GPS1.rel_alt)

	NED_rel_vel.x = float(inp_GPS2.vx - inp_GPS1.vx)
	NED_rel_vel.y = float(inp_GPS2.vy - inp_GPS1.vy)
	NED_rel_vel.z = float(inp_GPS2.vz - inp_GPS1.vz)


def px4_com_DRN1():

        global GPS1, GPS2, jsn_GPS_This_Drone, jsn_GPS_Other_Drone, GPS1_Local_NED

	time_stmp = 0.0
	rospy.init_node("PX4_com_DRN1", anonymous=True)
	rate = rospy.Rate(100)
	rospy.on_shutdown(clean_shutdown)
	while not rospy.is_shutdown():
		rate.sleep()
		period = float(rospy.get_time()) - time_stmp
		time_stmp = float(rospy.get_time())
		freq = 1.0/period
		print("Frequency of PX4 communication node: %.2f" % freq)
		#--------------------------------------------------------------------------------------------
		#(START) receiving the GPS data of this drone from PX4
		msg = PX4.recv_match(type='GLOBAL_POSITION_INT',blocking=True)
		if not msg:
                        print("No global GPS data is received!")
			jsn_GPS_This_Drone = json.dumps({"lat":GPS1.lat,"lon":GPS1.lon,"alt":GPS1.alt,
				"rel_alt":GPS1.rel_alt,"vx":GPS1.vx,"vy":GPS1.vy,"vz":GPS1.vz,"hdg":GPS1.hdg})
		else:
			GPS1.lat = msg.lat
			GPS1.lon = msg.lon
			GPS1.alt = msg.alt
			GPS1.rel_alt = msg.relative_alt
			GPS1.vx = msg.vx
			GPS1.vy = msg.vy
			GPS1.vz = msg.vz
			GPS1.hdg = msg.hdg
			print("GPS position data: lattitude=%d --- longitude=%d --- altitude=%d" %
				(GPS1.alt,GPS1.lon,GPS1.alt))
			print("GPS velocity data: Vx=%d --- Vy=%d --- Vz=%d" % (GPS1.vx,GPS1.vy,GPS1.vz))
			print("GPS other data: relative_altitude:%d --- heading:%d" % (GPS1.rel_alt,GPS1.hdg))
			jsn_GPS_This_Drone = json.dumps({"lat":GPS1.lat,"lon":GPS1.lon,"alt":GPS1.alt,
				"rel_alt":GPS1.rel_alt,"vx":GPS1.vx,"vy":GPS1.vy,"vz":GPS1.vz,"hdg":GPS1.hdg})
			Results_File_DRN1.write("%.9f %d %d %d %d %d\r\n" % (rospy.get_time(),
				GPS1.lat,GPS1.lon,GPS1.alt,GPS1.rel_alt,GPS1.hdg))
                #......................................................................................................
                msg1 = PX4.recv_match(type='LOCAL_POSITION_NED',blocking=True)
                if not msg:
                        print("No local GPS data is received!")
                else:
                        GPS1_Local_NED.x = msg1.x
                        GPS1_Local_NED.y = msg1.y
                        GPS1_Local_NED.z = msg1.z
                        print("GPS local NED position data: x=%.3f --- y=%.3f --- z=%.3f" %
                                (GPS1_Local_NED.x,GPS1_Local_NED.y,GPS1_Local_NED.z))
                        Results_File_DRN1_Local.write("%.9f %.6f %.6f %.6f\r\n" % (rospy.get_time(),
                                GPS1_Local_NED.x,GPS1_Local_NED.y,GPS1_Local_NED.z))
		#(END) receiving the GPS data of the other drone via UDP
		#------------------------------------------------------------------------------------------
		#(START) recieving the GPS data of other drone via UDP + sending the GPS data of this drone
		try:
			This_Drone.sendto(jsn_GPS_This_Drone.encode(),Other_Drone)
			Resp, Addrs = This_Drone.recvfrom(10000)
			jsn_GPS_Other_Drone = json.loads(Resp.decode())
			GPS2.lat = int(jsn_GPS_Other_Drone.get("lat"))
			GPS2.lon = int(jsn_GPS_Other_Drone.get("lon"))
			GPS2.alt = int(jsn_GPS_Other_Drone.get("alt"))
			GPS2.rel_alt = int(jsn_GPS_Other_Drone.get("rel_alt"))
			GPS2.vx = int(jsn_GPS_Other_Drone.get("vx"))
			GPS2.vy = int(jsn_GPS_Other_Drone.get("vy"))
			GPS2.vz = int(jsn_GPS_Other_Drone.get("vz"))
			GPS2.hdg = int(jsn_GPS_Other_Drone.get("hdg"))
			Results_File_DRN2.write("%.9f %d %d %d %d %d\r\n" % (rospy.get_time(),
				GPS2.lat,GPS2.lon,GPS2.alt,GPS2.rel_alt,GPS2.hdg))
		except socket.timeout:
			print("No GPS data received from other drone!")
		#(END) receiving the GPS data of the other drone via UDP + sending the GPS data of this drone
		#-------------------------------------------------------------------------------------------
		#(START) Computing the relative position and velocity between the two drones
		GPS2NED_conversion(GPS1,GPS2)
		print("Relative position by GPS: NED_rel_pos_x=%.3f --- NED_rel_pos_y=%.3f --- NED_rel_pos_z=%.3f" %
			(NED_rel_pos.x,NED_rel_pos.y,NED_rel_pos.z))
		print("Relative velocity by GPS: NED_rel_vel_x=%.3f --- NED_rel_vel_y=%.3f --- NED_rel_vel_z=%.3f" %
			(NED_rel_vel.x,NED_rel_vel.y,NED_rel_vel.z))
		Results_File_REL.write("%.9f %.6f %.6f %.6f %.6f %.6f %.6f\r\n" % (rospy.get_time(),
			NED_rel_pos.x,NED_rel_pos.y,NED_rel_pos.z,NED_rel_vel.x,NED_rel_vel.y,NED_rel_vel.z))
		#(START) Computing the relative position and velocity between the two drones
		#--------------------------------------------------------------------------------------------

if __name__ == '__main__':
	try:
		px4_com_DRN1()
	except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
		pass

