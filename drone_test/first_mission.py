from pymavlink import mavutil
import cv2
import numpy as np
import time
from pyproj import Proj, Transformer

ara = mavutil.mavlink_connection('udpin:192.168.43.171:15550')

ara.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (ara.target_system, ara.target_component))

wgs84 = Proj(proj="latlong", datum="WGS84")
utm = Proj(proj="utm", zone=32, datum="WGS84")

transformer_to_cartesian = Transformer.from_proj(wgs84, utm)
transformer_to_geodetic = Transformer.from_proj(utm, wgs84)

def geodetic_to_cartesian(lat, lon, alt):
    easting, northing = transformer_to_cartesian.transform(lat, lon)
    return easting, northing, alt

def cartesian_to_geodetic(easting, northing, alt):
    lat, lon = transformer_to_geodetic.transform(easting, northing)
    return lat, lon, alt

def takeoff(ara, alt):
    ara.mav.command_long_send(
        ara.target_system,
        ara.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,                        
        0, 0, 0,
        0, 0,
        0,
        alt 
    )
    
def landing(ara):
    ara.mav.command_long_send(
        ara.target_system,
        ara.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0,
        0,
        0, 0,
        0
    )

def get_current_position(ara):
    msg = ara.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat / 1e7  # degrees
    lon = msg.lon / 1e7  # degrees
    alt = msg.alt / 1e3  # meters
    return lat, lon, alt

def graze_mission(ara, wanted_position, sheep_position):
    a, b, alt = geodetic_to_cartesian(*get_current_position(ara))
    w_lat, w_lon, w_alt = geodetic_to_cartesian(*wanted_position)
    s_lat, s_lon, s_alt = geodetic_to_cartesian(*sheep_position)
    
    n_lat, n_lon, n_alt = (2*w_lat-s_lat, 2*w_lon-s_lon, alt)
    
    return (cartesian_to_geodetic(n_lat, n_lon, n_alt), cartesian_to_geodetic(w_lat, w_lon, w_alt))

def send_waypoint(ara, lat, lon, alt, vx=0, vy=0, vz=0):
    ara.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        10,
        ara.target_system, ara.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000),
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        vx, vy, vz,
        0, 0, 0,
        0, 0))
    
def is_at_waypoint(current_position, waypoint, tolerance=0.01):
    current_lat, current_lon, current_alt = current_position
    target_lat, target_lon, target_alt = waypoint
    
    distance = np.sqrt(
        (current_lat - target_lat) ** 2 +
        (current_lon - target_lon) ** 2 +
        (current_alt - target_alt) ** 2
    )
    
    return distance <= tolerance

ara.arducopter_arm()
print("Arming motors")
ara.motors_armed_wait()
print("Motors armed")
ara.set_mode("GUIDED")
print("Setting mode to GUIDED")
ara.wait_heartbeat()
takeoff(ara, 5)
print("Going to takeoff")

time.sleep(20)

next_position = (60.411404, 10.486600, 5.0)

sheep_position = (60.410949, 10.491192, 5.0)

pogs = graze_mission(ara, next_position, sheep_position)

for pog in pogs:
    send_waypoint(ara, *pog)
    while not is_at_waypoint(get_current_position(ara), pog):
        print('flying', pog)