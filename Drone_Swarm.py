from pymavlink import mavutil
import time

the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

the_connection2 = mavutil.mavlink_connection('tcp:localhost:5772')
the_connection2.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection2.target_system, the_connection2.target_component))

the_connection3 = mavutil.mavlink_connection('tcp:localhost:5782')
the_connection3.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection3.target_system, the_connection3.target_component))

connections=[the_connection,the_connection2,the_connection3]

ALTITUDE=10


def Drone_Arm():
	for i in connections:
		i.mav.command_long_send(i.target_system, i.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,1,0,0,0,0,0,0)

def Drone_Disarm():
	for i in connections:
		i.mav.command_long_send(i.target_system, i.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,0,0,0,0,0,0,0)


def Drone_TakeOff():
	# speak("TakeOff")
	for i in connections:
		i.mav.command_long_send(i.target_system, i.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF	, 0,0,0,0,0,0,0,ALTITUDE)
	
def Drone_Mode(mode):
	#speak("RTL")
	# Get mode ID
	
	for i in connections:
		mode_id = i.mode_mapping()[mode]
		i.mav.set_mode_send(
		i.target_system,
		mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
		mode_id)
	
	
def Waypoint1():
	#speak("WP1")
	x=-100
	y=60
	for i in connections:
		i.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,i.target_system,i.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,3576,x,y,-25,0,0,0,0,0,0,0,10))
		print(i)
		x+=10
		y+=10


def Waypoint2():
	#speak("WP2")
	x=200
	y=-90
	for i in connections:
		i.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,i.target_system,i.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,3576,x,y,-25,0,0,0,0,0,0,0,10))
		print(i)
		x+=10
		y+=10

Drone_Mode('GUIDED')
Drone_Arm()
Drone_TakeOff()
time.sleep(15)
Waypoint1()
time.sleep(20)
Waypoint2()
time.sleep(20)
Drone_Mode('RTL')
time.sleep(15)
Drone_Disarm()




