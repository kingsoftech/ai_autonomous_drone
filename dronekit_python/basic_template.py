from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import argparse
import math


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


vehicle = connectCopter()

arm_and_takeOff(10)

