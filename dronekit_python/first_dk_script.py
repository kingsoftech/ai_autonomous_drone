from dronekit import connect,VehicleMode
#import api
import time

# Connect to UDP endpoint.
vehicle = connect('127.0.0.1:14550', wait_ready=True)
# Use returned Vehicle object to query device state - e.g. to get the mode:
print("Mode: %s" % vehicle.mode.name)
# vehicle is an instance of the Vehicle class
print("Autopilot Firmware version: %s" % vehicle.version)
#print ("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
print ("Global Location: %s" % vehicle.location.global_frame)
print ("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print ("Local Location: %s" % vehicle.location.local_frame)    #NED
print ("Attitude: %s" % vehicle.attitude)
print ("Velocity: %s" % vehicle.velocity)
print ("GPS: %s" % vehicle.gps_0)
print ("Groundspeed: %s" % vehicle.groundspeed)
print ("Airspeed: %s" % vehicle.airspeed)
print ("Gimbal status: %s" % vehicle.gimbal)
print ("Battery: %s" % vehicle.battery)
print ("EKF OK?: %s" % vehicle.ekf_ok)
print ("Last Heartbeat: %s" % vehicle.last_heartbeat)
print ("Rangefinder: %s" % vehicle.rangefinder)
print ("Rangefinder distance: %s" % vehicle.rangefinder.distance)
print ("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
print ("Heading: %s" % vehicle.heading)
print("Is Armable?: %s" % vehicle.is_armable)
print("System status: %s" % vehicle.system_status.state)
print("Mode: %s" % vehicle.mode.name)    # settable
print("Armed: %s" % vehicle.armed)    # settable
#disarm the vehicle
vehicle.armed = False

#set the default groundspeed to be used in movement commands
vehicle.groundspeed = 6

print ("Airspeed: %s" % vehicle.groundspeed)

print("Armed: %s" % vehicle.armed)    # settable

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.mode.name=='GUIDED' and not vehicle.armed:
    print(" Getting ready to take off ...")
    time.sleep(1)
 #Callback to print the location in global frames. 'value' is the updated value
def location_callback(self, attr_name, value):
     print("Location (Global): ", value)


 # Add a callback `location_callback` for the `global_frame` attribute.
vehicle.add_attribute_listener('location.global_frame', location_callback)

 # Wait 2s so callback can be notified before the observer is removed
time.sleep(2)

 # Remove observer - specifying the attribute and previously registered callback function
vehicle.remove_message_listener('location.global_frame', location_callback)

last_rangefinder_distance=0

@vehicle.on_attribute('rangefinder')
def rangefinder_callback(self,attr_name):
     #attr_name not used here.
     global last_rangefinder_distance
     if last_rangefinder_distance == round(self.rangefinder.distance, 1):
         return
     last_rangefinder_distance = round(self.rangefinder.distance, 1)
     print(" Rangefinder (metres): %s" % last_rangefinder_distance)
     
# Demonstrate getting callback on any attribute change
def wildcard_callback(self, attr_name, value):
    print(" CALLBACK: (%s): %s" % (attr_name,value))

print("\nAdd attribute callback detecting any attribute change")
vehicle.add_attribute_listener('*', wildcard_callback)


print("Wait 1s so callback invoked before observer removed")
time.sleep(1)

print(" Remove Vehicle attribute observer")
# Remove observer added with `add_attribute_listener()`
vehicle.remove_attribute_listener('*', wildcard_callback)

while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print(" Waiting for home location ...")

# We have a home location.
print ("\n Home location: %s" % vehicle.home_location)