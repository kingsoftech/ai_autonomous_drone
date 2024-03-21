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
vehicle = connectCopter()

#vehicle version
vehicle.wait_ready('autopilot_version')
print("autopilot version: %s"%vehicle.version)

#does the firmware support setting altitude by companion computer
print("support set attitude from companion: %s"%vehicle.capabilities.set_attitude_target_local_ned)


#read actual position
print("position: %s"%vehicle.location.global_relative_frame)

#read actual attitude roll, pitch, yaw
print("Attitude: %s"%vehicle.attitude)

#read actual velocity(m/s)
print("velocity: %s"%vehicle.velocity)

#did the vehicle receive the last heartbeat
print("last Heartbeat: %s"%vehicle.last_heartbeat)

#is the vehicle good to arm
print("Is the vehicle armable: %s"%vehicle.is_armable)

print("Ground speed: %s"%vehicle.groundspeed)

print("mode: %s"%vehicle.mode)

print("Armed: %s"%vehicle.armed)

print("EKF ok: %s"%vehicle.ekf_ok)

vehicle.close()


