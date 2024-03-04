from dronekit import connect

##Connecting the drone to script(Using UDP end point)

vehicle = connect('127.0.0.1:14550', wait_ready=True)