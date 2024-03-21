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
def  get_distance_meter(target_location, current_location):
    dLat = target_location.lat - current_location.lat
    dLon = target_location.lon - current_location.lon

    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def move_coordinate(lat, lon, distance):
    # Earth radius in meters
    earth_radius = 6378137.0
    
    # Offset in radians
    dLat = distance / earth_radius
    dLon = distance / (earth_radius * math.cos(math.pi * lat / 180))
    
    # New latitude and longitude
    new_lat = lat + (dLat * 180 / math.pi)
    new_lon = lon + (dLon * 180 / math.pi)
    
    return new_lat, new_lon

def goto(targetLocation):
    print("implementing goto function")
    distance_to_target_location = get_distance_meter(targetLocation, vehicle.location.global_relative_frame) 
    vehicle.simple_goto(targetLocation)
    
    while vehicle.mode.name == "GUIDED":
        currentDistance =get_distance_meter(targetLocation, vehicle.location.global_relative_frame)
        if currentDistance <distance_to_target_location*.01:
            print("Reached target waypoint")
            time.sleep(2)
            break
    time.sleep(1)
    return None
def land():
    vehicle.mode = VehicleMode("LAND")
    while vehicle.mode.name != "LAND": 
        print("waiting for vehicle to enter land mode")
        time.sleep(1)  
    print("Vehicle landing!!!")
vehicle = connectCopter()

copter_current_location = vehicle.location.global_relative_frame

goto_lat1, goto_lon1 = move_coordinate(copter_current_location.lat, copter_current_location.lon, 10)

wp1 = LocationGlobalRelative(goto_lat1, goto_lon1, 10)

arm_and_takeOff(10)

goto(wp1)

land()