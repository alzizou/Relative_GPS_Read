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

Other_Drone = ('192.168.3.226',8100)

This_Drone2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
This_Drone2.bind(('192.168.3.184',8000))
This_Drone2.settimeout(0.1)

Other_Drone2 = ('192.168.3.226',8000)

Results_File_REL = open(r"/home/ubuntu/ali_ws/logs/Results_File_REL.txt","w+")
Results_File_DRN1 = open(r"/home/ubuntu/ali_ws/logs/Results_File_DRN1.txt","w+")
Results_File_DRN2 = open(r"/home/ubuntu/ali_ws/logs/Results_File_DRN2.txt","w+")

Results_File_REL_est = open(r"/home/ubuntu/ali_ws/logs/Results_File_REL_est.txt","w+")
Results_File_DRN1_est = open(r"/home/ubuntu/ali_ws/logs/Results_File_DRN1_est.txt","w+")
Results_File_DRN2_est = open(r"/home/ubuntu/ali_ws/logs/Results_File_DRN2_est.txt","w+")

class GPS_data:
	time_usec = 0
	fix_type = 0
	lat = 0
	lon = 0
	alt = 0
	eph = 0
	epv = 0
	vel = 0
	cog = 0
	satellites_visible = 0
	vx=0
	vy=0

class Global_Position_Estimate_data:
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

Global_Position_Estimate_1 = Global_Position_Estimate_data()
Global_Position_Estimate_2 = Global_Position_Estimate_data()

class NED_vector:
	x=0.0
	y=0.0
	z=0.0

NED_rel_pos = NED_vector()
NED_rel_vel = NED_vector()


jsn_GPS_This_Drone = {}
jsn_GPS_Other_Drone = {}

jsn_Global_Position_Estimate_This_Drone = {}
jsn_Global_Position_Estimate_Other_Drone = {}

def clean_shutdown():
	Results_File_REL.close()
	Results_File_DRN1.close()
	Results_File_DRN2.close()
	Results_File_REL_est.close()
	Results_File_DRN1_est.close()
	Results_File_DRN2_est.close()
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
	NED_rel_pos.z = - float(inp_GPS2.alt - inp_GPS1.alt)
	
	NED_rel_vel.x = float(inp_GPS2.vx - inp_GPS1.vx)
	NED_rel_vel.y = float(inp_GPS2.vy - inp_GPS1.vy)
	#NED_rel_vel.z = float(inp_GPS2.vz - inp_GPS1.vz)


def px4_com_DRN1():

	global GPS1, GPS2, jsn_GPS_This_Drone, jsn_GPS_Other_Drone
	global Global_Position_Estimate_1, Global_Position_Estimate_2, jsn_Global_Position_Estimate_This_Drone, jsn_Global_Position_Estimate_Other_Drone

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
		msg = PX4.recv_match(type='GPS_RAW_INT',blocking=True)
		if not msg:
			print("No GPS data is received!")
			jsn_GPS_This_Drone = json.dumps({"time_usec":GPS1.time_usec,"fix_type":GPS1.fix_type,"lat":GPS1.lat,"lon":GPS1.lon,"alt":GPS1.alt,
				"eph":GPS1.eph,"epv":GPS1.epv,"vel":GPS1.vel,"cog":GPS1.cog,"satellites_visible":GPS1.satellites_visible})
		else:
			# GPS_RAW_INT (#24), Type, Units, Description
			GPS1.time_usec = msg.time_usec # uint64_t, us, Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
			GPS1.fix_type = msg.fix_type # uint8_t, GPS fix type.
			GPS1.lat = msg.lat # int32_t, degE7, Latitude (WGS84, EGM96 ellipsoid)
			GPS1.lon = msg.lon # int32_t, degE7, Longitude (WGS84, EGM96 ellipsoid)
			GPS1.alt = msg.alt # int32_t, mm, Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
			GPS1.eph = msg.eph # uint16_t, GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
			GPS1.epv = msg.epv # uint16_t, GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
			GPS1.vel = msg.vel # uint16_t, cm/s, GPS ground speed. If unknown, set to: UINT16_MAX
			GPS1.cog = msg.cog # uint16_t, cdeg, Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
			GPS1.satellites_visible = msg.satellites_visible # uint8_t, Number of satellites visible. If unknown, set to 255

			#Use speed and course over ground to estimate vx,vy
			GPS1.vx = GPS1.vel * mt.cos(GPS1.cog*3.1415/18000.0) / 10.0
			GPS1.vy = GPS1.vel * mt.sin(GPS1.cog*3.1415/18000.0) / 10.0

			print("GPS position data: lattitude=%d --- longitude=%d --- altitude=%d" %
				(GPS1.alt,GPS1.lon,GPS1.alt))

			print("GPS velocity data: vel=%d --- cog=%d" % (GPS1.vel,GPS1.cog))
			jsn_GPS_This_Drone = json.dumps({"time_usec":GPS1.time_usec,"fix_type":GPS1.fix_type,"lat":GPS1.lat,"lon":GPS1.lon,"alt":GPS1.alt,
				"eph":GPS1.eph,"epv":GPS1.epv,"vel":GPS1.vel,"cog":GPS1.cog,"satellites_visible":GPS1.satellites_visible, "vx":GPS1.vx, "vy":GPS1.vy})
			Results_File_DRN1.write("%.9f %d %d %d %d %d %d %d \r\n" % (rospy.get_time(),
				GPS1.lat,GPS1.lon,GPS1.alt,GPS1.vel,GPS1.cog,GPS1.fix_type,GPS1.satellites_visible))
		#--------------------------------------------------------------------------------------------
		msg_est = PX4.recv_match(type='GLOBAL_POSITION_INT',blocking=True)
		if not msg_est:
			print("No global position estimate data is received!")
			jsn_Global_Position_Estimate_This_Drone = json.dumps({"lat":Global_Position_Estimate_1.lat,"lon":Global_Position_Estimate_1.lon,"alt":Global_Position_Estimate_1.alt,
				"rel_alt":Global_Position_Estimate_1.rel_alt,"vx":Global_Position_Estimate_1.vx,"vy":Global_Position_Estimate_1.vy,"vz":Global_Position_Estimate_1.vz,"hdg":Global_Position_Estimate_1.hdg})
		else:
			Global_Position_Estimate_1.lat = msg_est.lat
			Global_Position_Estimate_1.lon = msg_est.lon
			Global_Position_Estimate_1.alt = msg_est.alt
			Global_Position_Estimate_1.rel_alt = msg_est.relative_alt
			Global_Position_Estimate_1.vx = msg_est.vx
			Global_Position_Estimate_1.vy = msg_est.vy
			Global_Position_Estimate_1.vz = msg_est.vz
			Global_Position_Estimate_1.hdg = msg_est.hdg
			print("GPS position data: lattitude=%d --- longitude=%d --- altitude=%d" %
				(Global_Position_Estimate_1.lat,Global_Position_Estimate_1.lon,Global_Position_Estimate_1.alt))
			print("GPS velocity data: Vx=%d --- Vy=%d --- Vz=%d" % (Global_Position_Estimate_1.vx,Global_Position_Estimate_1.vy,Global_Position_Estimate_1.vz))
			print("GPS other data: relative_altitude:%d --- heading:%d" % (Global_Position_Estimate_1.rel_alt,Global_Position_Estimate_1.hdg))
			jsn_Global_Position_Estimate_This_Drone = json.dumps({"lat":Global_Position_Estimate_1.lat,"lon":Global_Position_Estimate_1.lon,"alt":Global_Position_Estimate_1.alt,
				"rel_alt":Global_Position_Estimate_1.rel_alt,"vx":Global_Position_Estimate_1.vx,"vy":Global_Position_Estimate_1.vy,"vz":Global_Position_Estimate_1.vz,"hdg":Global_Position_Estimate_1.hdg})
			Results_File_DRN1_est.write("%.9f %d %d %d %d %d\r\n" % (rospy.get_time(),
				Global_Position_Estimate_1.lat,Global_Position_Estimate_1.lon,Global_Position_Estimate_1.alt,Global_Position_Estimate_1.rel_alt,Global_Position_Estimate_1.hdg))

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
			GPS2.vel = int(jsn_GPS_Other_Drone.get("vel"))
			GPS2.cog = int(jsn_GPS_Other_Drone.get("cog"))
			GPS2.fix_type = int(jsn_GPS_Other_Drone.get("fix_type"))
			GPS2.satellites_visible = int(jsn_GPS_Other_Drone.get("satellites_visible"))
			Results_File_DRN2.write("%.9f %d %d %d %d %d %d %d\r\n" % (rospy.get_time(),
				GPS2.lat,GPS2.lon,GPS2.alt,GPS2.vel,GPS2.cog,GPS2.fix_type,GPS2.satellites_visible))
			#Use speed and course over ground to estimate vx,vy
			GPS2.vx = GPS2.vel * mt.cos(GPS2.cog*3.1415/18000.0) / 10.0
			GPS2.vy = GPS2.vel * mt.sin(GPS2.cog*3.1415/18000.0) / 10.0
		except socket.timeout:
			print("No GPS data received from other drone!")
		#(END) receiving the GPS data of the other drone via UDP + sending the GPS data of this drone
		#-------------------------------------------------------------------------------------------
		#(START) Computing the relative position and velocity between the two drones
		GPS2NED_conversion(GPS1,GPS2)
		print("Relative position by GPS: NED_rel_pos_x=%.3f --- NED_rel_pos_y=%.3f --- NED_rel_pos_z=%.3f" %
			(NED_rel_pos.x,NED_rel_pos.y,NED_rel_pos.z))
		print("Relative velocity by GPS: NED_rel_vel_x=%.3f --- NED_rel_vel_y=%.3f"
			% (NED_rel_vel.x,NED_rel_vel.y))
		Results_File_REL.write("%.9f %.6f %.6f %.6f %.6f %.6f \r\n" % (rospy.get_time(),
			NED_rel_pos.x,NED_rel_pos.y,NED_rel_pos.z,NED_rel_vel.x,NED_rel_vel.y))
		#--------------------------------------------------------------------------------------------
		#(START) recieving the global position estimate data of other drone via UDP + sending the Global position estimate data of this drone  
		try:
			This_Drone2.sendto(jsn_Global_Position_Estimate_This_Drone.encode(),Other_Drone2)
			Resp, Addrs = This_Drone2.recvfrom(10000)
			jsn_Global_Position_Estimate_Other_Drone = json.loads(Resp.decode())
			Global_Position_Estimate_2.lat = int(jsn_Global_Position_Estimate_Other_Drone.get("lat"))
			Global_Position_Estimate_2.lon = int(jsn_Global_Position_Estimate_Other_Drone.get("lon"))
			Global_Position_Estimate_2.alt = int(jsn_Global_Position_Estimate_Other_Drone.get("alt"))
			Global_Position_Estimate_2.rel_alt = int(jsn_Global_Position_Estimate_Other_Drone.get("rel_alt"))
			Global_Position_Estimate_2.vx = int(jsn_Global_Position_Estimate_Other_Drone.get("vx"))
			Global_Position_Estimate_2.vy = int(jsn_Global_Position_Estimate_Other_Drone.get("vy"))
			Global_Position_Estimate_2.vz = int(jsn_Global_Position_Estimate_Other_Drone.get("vz"))
			Global_Position_Estimate_2.hdg = int(jsn_Global_Position_Estimate_Other_Drone.get("hdg"))
			Results_File_DRN2_est.write("%.9f %d %d %d %d %d\r\n" % (rospy.get_time(),
				Global_Position_Estimate_2.lat,Global_Position_Estimate_2.lon,Global_Position_Estimate_2.alt,Global_Position_Estimate_2.rel_alt,Global_Position_Estimate_2.hdg))
		except socket.timeout:
			print("No global position estimate data received from other drone!")
		#(END) receiving the GPS data of the other drone via UDP + sending the GPS data of this drone
		#-------------------------------------------------------------------------------------------
		#(START) Computing the relative position and velocity between the two drones
		GPS2NED_conversion(Global_Position_Estimate_1,Global_Position_Estimate_2)
		print("Relative position by GPS: NED_rel_pos_x=%.3f --- NED_rel_pos_y=%.3f --- NED_rel_pos_z=%.3f" %
			(NED_rel_pos.x,NED_rel_pos.y,NED_rel_pos.z))
		print("Relative velocity by GPS: NED_rel_vel_x=%.3f --- NED_rel_vel_y=%.3f" %
			(NED_rel_vel.x,NED_rel_vel.y))
		Results_File_REL_est.write("%.9f %.6f %.6f %.6f %.6f %.6f\r\n" % (rospy.get_time(),
			NED_rel_pos.x,NED_rel_pos.y,NED_rel_pos.z,NED_rel_vel.x,NED_rel_vel.y))
		#--------------------------------------------------------------------------------------------

if __name__ == '__main__':
	try:
		px4_com_DRN1()
	except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
		pass

