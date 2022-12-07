from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, APIException
from pymavlink import mavutil
import time
import socket
import math
import argparse

#Print vehicle Informations on CMD
def printinformation(vehicle):
    attitude_pitch=vehicle.attitude.pitch
    attitude_roll=vehicle.attitude.roll
    attitude_yaw=vehicle.attitude.yaw
    altitude=vehicle.location.global_relative_frame.alt
    print("altitude: " + str(altitude)+" pitch: " + str(attitude_pitch)+" roll: "+str(attitude_roll)+" yaw: "+str(attitude_yaw))

# Connect to the drone
def connectMyCopter():
    parser=argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect',default='127.0.0.1:14550')
    args=parser.parse_args()
    print("Connecting to the drone...")
    connection_string=args.connect

    try:
        vehicle=connect(connection_string,wait_ready=True)
    # Bad TCP connection
    except socket.error:
        print ("No server exists!")

    # API Error
    except APIException:
        print ("Timeout!")

    # Other error
    except:
        print ("Some other error!")
    return vehicle

# Simple function to arm and takeoff to a target altitude
def arm_and_takeoff(vehicle,TargetAltitude):
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable")
        time.sleep(1)

    #Switch vehicle to GUIDED mode and wait for change
    vehicle.mode=VehicleMode("GUIDED")
    while vehicle.mode!="GUIDED":
        print("Waiting for vehicle to enter GUIDED mode")
        time.sleep(1)

    # Arm vehicle
    vehicle.armed=True
    while vehicle.armed==False:
        print("Waiting for vehicle to become armed")
        time.sleep(1)

    vehicle.simple_takeoff(TargetAltitude)

    while True:
        printinformation(vehicle)
        if vehicle.location.global_relative_frame.alt>= TargetAltitude*0.95:
            break
        time.sleep(0.5)

    print("Target altitude reached")
    return None

# Land with the vehicle
def land(vehicle):
    set_velocity_body(vehicle)
    set_yaw_body(vehicle)
    time.sleep(1)
    vehicle.mode=VehicleMode("LAND")
    while vehicle.location.global_relative_frame.alt >= 0.5 :
        printinformation(vehicle)
        time.sleep(0.3)
    time.sleep(2)

# Send velocity commands to the drone
def set_velocity_body(vehicle,Vx=0,Vy=0,Vz=0):
    msg=vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b100111000111, # Bitmask-> consider only velocities
        0, 0, 0, # Position
        Vx, Vy, Vz, # Velocity
        0, 0, 0, # Accel
        0, 0)  #Yaw
    vehicle.send_mavlink(msg)
    vehicle.commands.upload()

# Send yaw commands to the drone
def set_yaw_body(vehicle,heading=0,direction=1,relative=True):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        direction,  # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.commands.upload()

# Sends Vx, Vy and Yaw values to the drone
def setvelocitywithyawangle(vehicle,constfwspeed=0.1,yerror=0,erroryaw=0):
    if yerror==0:
        normalizederror=0
    else:
        normalizederror=0.01*yerror
    set_velocity_body(vehicle,constfwspeed,normalizederror,0)
    if erroryaw<0:
        erroryaw= erroryaw*-1
        direction=-1
    else:
        direction=1
    set_yaw_body(vehicle,erroryaw,direction)

# Mavlink testing misssion
def mavlink_mission(vehicle):
    counter = 0
    while counter<2:
        set_velocity_body(vehicle,0.5,0,0)
        print("Direction: NORTH relative to heading of drone")
        time.sleep(1)
        counter= counter +1

    set_velocity_body(vehicle)
    time.sleep(1)
    counter = 0
    while counter<2:
        set_velocity_body(vehicle,-0.5,0,0)
        print("Direction: SOUTH relative to heading of drone")
        time.sleep(1)
        counter= counter +1

    set_velocity_body(vehicle)
    time.sleep(1)
    counter = 0
    while counter<2:
        set_velocity_body(vehicle,0,0.5,0)
        print("Direction: EAST relative to heading of drone")
        time.sleep(1)
        counter= counter +1

    set_velocity_body(vehicle)
    time.sleep(1)
    counter = 0
    
    while counter<2:
        set_velocity_body(vehicle,0,-0.5,0)
        print("Direction: WEST relative to heading of drone")
        time.sleep(1)
        counter= counter +1

    set_velocity_body(vehicle)
###MAIN Mission
if __name__=="__main__":
    errorarray=[-10,-7,-5,-2,0,5,1,2,5,10,10,6,3,-6,-3,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    weights=[-25,-15,0,15,25]
    erroryawarray=[2,2,2,2,2,2,2,2,2,2,2,2,0,2,2,2,2,1,2,2,4,4,1,0,0,1,1,2,3,2,4,3,2,1]

    errorarray2=[-10,-7,-5,-2,0,0,0,2,5,10,10,6,3,0,0,-1,0,1,0,2,5,3,0,-3,-5,-8,-5,-3,-4,-2,0,0,1,0]
    erroryawarray2=[0,1,2,2,2,2,2,4,3,2,2,1,2,2,1,0,0,1,2,2,4,4,3,3,2,2,2,2,1,2,4,3,2,1]
    vehicle=None
    # Connect to the drone
    vehicle=connectMyCopter()
    # vehicle Informations on CMD
    print(vehicle)
    print(vehicle.battery)
    print("Takeoff in...")
    for i in range(5):
        print(5-i)
        time.sleep(1)
    try:
        arm_and_takeoff(vehicle,1.5)
        time.sleep(2)
        print("Starting Simple Mission...")
        # Testing only translation or yaw
        for i in range(len(errorarray)):
            setvelocitywithyawangle(vehicle,yerror=errorarray[i],erroryaw=weights[erroryawarray[i]])
            print(errorarray[i],weights[erroryawarray[i]])
            time.sleep(0.5)
        #print("Second phase comming")
        set_velocity_body(vehicle)
        #time.sleep(2)
        mavlink_mission(vehicle)
        time.sleep(2)
        # Test both translation and yaw
        for i in range(len(errorarray2)):
            setvelocitywithyawangle(vehicle,yerror=errorarray2[i],erroryaw=weights[erroryawarray2[i]])
            print(errorarray2[i],weights[erroryawarray2[i]])
            time.sleep(0.5)
    finally:
        if vehicle!=None:
            print("Landing")
            land(vehicle)
            vehicle.close()
