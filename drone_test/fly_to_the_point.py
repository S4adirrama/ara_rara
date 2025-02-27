from pymavlink import mavutil

ara = mavutil.mavlink_connection('udpin:10.1.90.114:15550')

ara.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (ara.target_system, ara.target_component))

target_lat = 47.397742
target_lon = 8.545594
target_alt = 10

def send_waypoint(ara, lat, lon, alt):
    ara.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        10,
        ara.target_system, ara.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000),
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0))


ara.arducopter_arm()
print("Arming motors")
ara.motors_armed_wait()
print("Motors armed")

ara.set_mode("GUIDED")
print("Setting mode to GUIDED")
ara.wait_heartbeat()

send_waypoint(ara, target_lat, target_lon, target_alt)
print(f"Going to location: lat={target_lat}, lon={target_lon}, alt={target_alt}")


import time
time.sleep(30)