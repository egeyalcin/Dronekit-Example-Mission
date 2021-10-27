##########DEPENDENCIES#############
from dronekit import connect, VehicleMode,LocationGlobalRelative, LocationGlobal, APIException
from pymavlink import mavutil
import time
import socket
import math
import argparse
import cv2
import sys
import numpy as np
import datetime



#########FUNCTIONS#################
parser = argparse.ArgumentParser()
parser.add_argument('--connect',default = "127.0.0.1:14550",help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

def arm_and_takeoff(targetHeight):
	while vehicle.is_armable!=True:
		print("Arm olmadi")
		time.sleep(1)

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode!='GUIDED':
		print("GUIDED mod")
		time.sleep(1)
	print("GUIDED gecis basarili")

	vehicle.armed = True
    
	while vehicle.armed==False:
		time.sleep(1)
	print("kalkiyorum")

	vehicle.simple_takeoff(targetHeight) ##meters

	while True:
		print("yukseklik: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
			break
		time.sleep(1)
	print("hedefe ulastim")

	return None

def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def set_roi(location):
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)

def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;


def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;

def goto_position_target_global_int(aLocation):
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)



def goto_position_target_local_ned(north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)



def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto): 
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("hedefe uzaklik: ", remainingDistance)
        if remainingDistance<=targetDistance*0.095: #buradaki degeri 0.01 idi ben 0.1 yaptim hasasligi azltmak icin
            print("hedefe vardim")
            break;
        time.sleep(2)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    
    


def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def get_distance_meters(targetLocation,currentLocation):
	dLat=targetLocation.lat - currentLocation.lat
	dLon=targetLocation.lon - currentLocation.lon
	
	return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def gotokor(targetLocation):
	distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

	vehicle.simple_goto(targetLocation)

	while vehicle.mode.name=="GUIDED":
		currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
		if currentDistance<distanceToTargetLocation*.01:
			print("Reached target waypoint.")
			time.sleep(2)
			break
		time.sleep(1)
	return None   

def send_local_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def send_land_message(x,y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,0,0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush() 

def kamera():
    cap = cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)
    tarih = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    dosya_ismi = tarih + '.avi'
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    kayit = cv2.VideoWriter(dosya_ismi,fourcc,24.0,(640,480))


    while cap.isOpened():
        _,imgOriginal = cap.read()
        kayit.write(imgOriginal)
        blurred = cv2.GaussianBlur(imgOriginal,(11,11),0)  
        hsv = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)

        lower_red = np.array([0,120,70])
        upper_red = np.array([10,255,255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170,120,70])
        upper_red = np.array([180,255,255])
        mask2 = cv2.inRange(hsv,lower_red,upper_red)
        
        mask = mask1+mask2
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((3,3),np.uint8))

        contours,_ = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        center = None
        
        if len(contours)>0:
           
            c = max(contours,key=cv2.contourArea)
           
            rectangle = cv2.minAreaRect(c)
            
            ((x,y),(width,height),(rotation)) = rectangle 

            area = int(width*height)
            if area >5000:
                s = "x: {}, y: {}, width: {}, height: {}, rotation: {}".format(np.round(x),np.round(y),np.round(width),np.round(height),np.round(rotation))
                
                   
                box = cv2.boxPoints(rectangle)
                box = np.int64(box)

                  
                M = cv2.moments(c)
                center = (int(M["m10"]/M["m00"]),int(M["m01"]/M["m00"])) 
                center_x,center_y = center  
                 

      

                x_ang = (center_x - 640*.5)*(62.2 * (math.pi / 180 )/640)
                y_ang = (center_y- 480*.5)*(48.8 * (math.pi / 180)/480)
    
        
                if vehicle.location.global_relative_frame.alt >= 5:
                    if vehicle.mode!='LAND':
                        vehicle.mode = VehicleMode('LAND')
                        while vehicle.mode!='LAND':
                            time.sleep(1)     
                            send_land_message(x_ang,y_ang)
                    else:
                        send_land_message(x_ang,y_ang)
                        pass
                else:
                    vehicle.mode = VehicleMode('GUIDED')
                    x = vehicle.location.global_relative_frame.lat
                    y = vehicle.location.global_relative_frame.lon 
                    print(x,y)  
                    time.sleep(1)    
                    vehicle.mode = VehicleMode('LAND') 
                    kayit.release()
                    cap.release()

                    
       

##########MAIN EXECUTABLE###########
if __name__ == '__main__':
    arm_and_takeoff(10)
    #goto(5,0, goto_position_target_global_int) #kuzeye gider
    #condition_yaw(270)
    #goto(0,-5, goto_position_target_global_int) #batiya gider
    #condition_yaw(180)
    #goto(-5,0, goto_position_target_global_int)
    #condition_yaw(90)
    #goto(0,5, goto_position_target_global_int)
    #condition_yaw(0)
    #goto(5,0, goto_position_target_global_int)
    vehicle.parameters['LAND_SPEED'] = 70
    kamera()
    # wp1 = LocationGlobalRelative(44.50202,-88.060316,2)
    # gotokor(wp1)

    #vehicle.mode = VehicleMode('LAND')



