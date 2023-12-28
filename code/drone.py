from pymavlink import mavutil
import commands
import socket
import time

class Drone:       
    def __init__(self, testing=False):
        if testing == True:
            connectionString = socket.gethostbyname_ex(socket.gethostname())[-1][1] + ":14550"
        else:
            connectionString = "/dev/ttyAMA0"
        print(connectionString)
        self.vehicle = mavutil.mavlink_connection(connectionString)
        self.vehicle.wait_heartbeat()
    
        commands.request_message_interval(self.vehicle, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 24)

    ## Mavlink
    def get_params(self):
        # get position and velocity data
        self.vehicle.altitude = -commands.get_data(self.vehicle, "LOCAL_POSITION_NED").get('z')
        self.vehicle.descent_rate = commands.get_data(self.vehicle, "LOCAL_POSITION_NED").get('vz')

    def takeoff(self, alt):
        commands.set_mode(self.vehicle, "GUIDED")
        commands.arm_throttle(self.vehicle)
        commands.takeoff(self.vehicle, alt=alt)

    def send_acceleration(self, freefall=False):
        a = self.calculate_target_acceleration() if not freefall else 10
        commands.send_acceleration(self.vehicle, a)

    def send_velocity(self, v):
        commands.send_velocity(self.vehicle, v)

    def send_position(self, x, y, z):
        commands.send_position(self.vehicle, x, y, z)
    
    def set_mode(self, mode):
        commands.set_mode(self.vehicle, mode)
    
    ## Flight Plan

    def calculate_target_acceleration(self):
        v1 = self.vehicle.descent_rate
        v2 = 0
        d = self.vehicle.altitude - 1

        a = ((v2**2)-(v1**2)) / (2*(d))
        
        print(f"current v: {v1}, d to ground: {d}, commanded accel: {a}")
        return a