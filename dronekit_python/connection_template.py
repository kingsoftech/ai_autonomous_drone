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