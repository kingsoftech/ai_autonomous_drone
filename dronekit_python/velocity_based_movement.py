from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import argparse
import math
from pymavlink import mavutil


def connectCopter():
    '''
    python3 connection_template.py --connect 127.0.0.1:14550
    '''
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    
    connection_string = args.connect
    vehicle = connect(connection_string,wait_ready=True)
    if not connection_string:
       vehicle = connect('127.0.0.1:14550',wait_ready=True) 
    
    return vehicle

def arm_and_takeOff(target_height):
    while vehicle.is_armable != True:
        print("waiting for vehicle to become armable")
        time.sleep(0)
    print("Vehicle is now armable")
    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != "GUIDED":
        print("waiting for drone to enter guided flight mode")
        time.sleep(0)
    print("Vehicle now in Guided flight mode")

    vehicle.armed = True

    while vehicle.armed == False:
        print("waiting for vwehicle to arm")
        time.sleep(0)
    print("vehicle is armed")
    
    print ("Taking off!")
    vehicle.simple_takeoff(target_height) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=target_height*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)
def send_local_ned_velocity(v_x,v_y,v_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        v_x,v_y,v_z,
        0, 0, 0,
        0, 0
    )
    
    vehicle.send_mavlink(msg)
    vehicle.flush()
def send_global_ned_velocity(v_x,v_y,v_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        v_x,v_y,v_z,
        0, 0, 0,
        0, 0
    )
    
    vehicle.send_mavlink(msg)
    vehicle.flush()  

vehicle = connectCopter()

arm_and_takeOff(3)

counter= 0
while counter<3:
    send_global_ned_velocity(2,0,0)
    time.sleep(1)
    print("Moving NORTH relative to the front of drone")
    counter +=1
    
time.sleep(2)

counter= 0
while counter<3:
    send_global_ned_velocity(0,-2,0)
    time.sleep(1)
    print("Moving WEST relative to the front of drone")
    counter +=1
    
counter= 0
while counter<3:
    send_local_ned_velocity(2,0,0)
    time.sleep(1)
    print("Moving TRUE NORTH relative to the front of drone")
    counter +=1
    
time.sleep(2)

counter= 0
while counter<3:
    send_local_ned_velocity(0,-2,0)
    time.sleep(1)
    print("Moving TRUE WEST relative to the front of drone")
    counter +=1
