from pymavlink import mavutil
import commands
import socket
import time

class Drone:       
    def __init__(self):
        connectionString = socket.gethostbyname_ex(socket.gethostname())[-1][1] + ":14550"
        #self.vehicle = mavutil.mavlink_connection(connectionString)
        self.vehicle.wait_heartbeat()
        print(connectionString)

        commands.request_message_interval(self.vehicle, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 24)
        commands.request_message_interval(self.vehicle, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 24)

    ## Mavlink
    def get_params(self):
        #get attitude data

        #get position data
        self.vehicle.altitude = -self.get_data(self.vehicle, "LOCAL_POSITION_NED").get('z')
        self.vehicle.descent_rate = self.get_data(self.vehicle, "LOCAL_POSITION_NED").get('vz')


    def takeoff(self, alt):
        commands.set_mode(self.vehicle, "GUIDED")
        commands.arm_throttle(self.vehicle)
        commands.takeoff(self.vehicle, alt=alt)

    def send_acceleration(self, freefall=False):
        
        a = self.calculate_target_acceleration() if not freefall else 10
        commands.send_acceleration(self.vehicle, a)
    
    def set_mode(self, mode:str):
        commands.set_mode(self.vehicle, mode)
    
    ## Flight Plan


    def calculate_target_acceleration(self):
        v1 = self.vehicle.descent_rate
        v2 = 0
        d = self.vehicle.altitude - 0.5

        #a = ((v2**2)-(v1**2)) / (2*(d))
        a = (v2-v1)/3
        
        print(f"current v: {v1}, d to ground: {d}, commanded accel: {a}")
        return a