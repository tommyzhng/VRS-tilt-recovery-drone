from drone import Drone
import time
import keyboard

drone = Drone(testing=True)
test_alt = 100

while True:
    drone.get_params()
    #if t is pressed, go to 100m
    if keyboard.is_pressed('t'):
        print("taking off")
        drone.set_mode("GUIDED")
        drone.takeoff(test_alt)

    #if g is pressed, go to 100m
    if keyboard.is_pressed('r'):
        print("sending position")
        drone.set_mode("GUIDED")
        drone.send_position(0, 0, test_alt)
    
    #if f is pressed, freefall
    if keyboard.is_pressed('f'):
        print("sending freefall")
        drone.set_mode("GUIDED")
        while True:
            drone.get_params()
            drone.send_acceleration() if drone.vehicle.altitude < 60 else drone.send_acceleration(freefall=True)
            if drone.vehicle.altitude < 0.5:
                print("stopped")
                drone.set_mode("LAND")
                break
            elif keyboard.is_pressed('q'):
                drone.send_velocity(0)
                print("stopped")
                break

    #if h is pressed, hover
    if keyboard.is_pressed('h'):
        print("hovering")
        drone.set_mode("GUIDED")
        drone.send_velocity(0)
    
    #if l is pressed, land
    if keyboard.is_pressed('l'):
        print("landing")
        drone.set_mode("LAND")
        break

