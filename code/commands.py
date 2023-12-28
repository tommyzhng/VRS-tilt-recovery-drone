from pymavlink import mavutil
import time

def get_data(vehicle, command):
        return vehicle.recv_match(type=command, blocking=True).to_dict()
    
def request_message_interval(vehicle, message_id: int, frequency_hz: float):
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id,
        1e6 / frequency_hz, # Interval in microseconds. -1 to disable and 0 to request default.
        0, 0, 0, 0, 
        0, # Target address. 0: Flight-stack , 1: address of requestor, 2: broadcast.
    )

def set_mode(vehicle, mode):
    mode_id = vehicle.mode_mapping()[mode]
    vehicle.set_mode(mode_id)

def arm_throttle(vehicle):
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 
        0, 0, 0, 0, 0, 0)
    print("Waiting for the vehicle to arm")
    vehicle.motors_armed_wait()
    print("Armed")

def takeoff(vehicle, alt):
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 
        0, 0, 0, 0, 0, 0, 
        alt)
    print("Taking off")

    while -get_data(vehicle, "LOCAL_POSITION_NED").get('z') < alt - 0.5:
        pass

    print("Reached target height")

def send_acceleration(vehicle, a):
    msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110000111111,
        0, 0, 0,
        0, 0, 0,
        0, 0, a,
        0, 0)
    vehicle.mav.send(msg) 

def send_velocity(vehicle, v):
    msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111000111,
        0, 0, 0,
        0, 0, v,
        0, 0, 0,
        0, 0)
    vehicle.mav.send(msg)

def send_position(vehicle, x, y, z):
    msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,
        x, y, -z,
        0, 0, 0,
        0, 0, 0,
        0, 0)
    vehicle.mav.send(msg)


def yaw_instruction(vehicle):
    if vehicle.location.global_relative_frame.alt < 10:
        a = vehicle.calculate_acceleration()
        v = vehicle.velocity[2]
    else:
        v = 3.5
        a = 0
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b010000000111,
        0, 0, 0, #position
        5, 0, v, #velocity
        0, 0, a, #acceleration
        0, 1.3)    #yaw
    vehicle.send_mavlink(msg)
    vehicle.request_data()
    time.sleep(0.2)