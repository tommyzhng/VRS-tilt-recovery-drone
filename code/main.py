from drone import Drone
from pymavlink import mavutil
from commands import get_data
import time

drone = Drone()
#drone.takeoff(50)
#drone.send_acceleration() if drone.vehicle.altitude <= 49 else drone.send_acceleration(freefall=True)

while True:
    drone.get_params()
    drone.send_acceleration()
    #if drone.vehicle.altitude <= 2:
       #break

drone.set_mode("LAND")