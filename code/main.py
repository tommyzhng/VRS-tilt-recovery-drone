from drone import Drone
from pymavlink import mavutil
import time

drone = Drone()
drone.set_mode("GUIDED")

drone.arm_throttle()
drone.takeoff(50)

while True:
    drone.vehicle.altitude = -drone.get_data("LOCAL_POSITION_NED", 20).get('z')
    drone.vehicle.descent_rate = -drone.get_data("LOCAL_POSITION_NED", 20).get('vz')
    drone.send_acceleration() if drone.vehicle.altitude <= 35 else drone.send_acceleration(freefall=True)
    if drone.vehicle.altitude <= 2:
       break
    
drone.set_mode("LAND")