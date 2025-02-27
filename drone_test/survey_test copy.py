from pymavlink import mavutil
from ultralytics import YOLO
import pygetwindow as gw
import cv2
import numpy as np
import time

model = YOLO("yolov10n.pt")
ara = mavutil.mavlink_connection('udpin:10.1.90.114:15550')
window = gw.getWindowsWithTitle("title")[0]

ara.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (ara.target_system, ara.target_component))

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

def survey(ara, x=10, y=10, c=4):
    cx, cy, cz = get_current_position(ara)
    
    x_points = np.linspace(cx - x, cx + x, c + 2)
    y_points = np.linspace(cy - y, cy + y, c + 2)
    
    points = []
    
    for x_val in x_points:
        for y_val in y_points:
            points.append((x_val, y_val, cz))

    return points

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
    
def is_at_waypoint(current_position, waypoint, tolerance=1.0):
    current_lat, current_lon, current_alt = current_position
    target_lat, target_lon, target_alt = waypoint
    
    distance = np.sqrt(
        (current_lat - target_lat) ** 2 +
        (current_lon - target_lon) ** 2 +
        (current_alt - target_alt) ** 2
    )
    
    return distance <= tolerance

def find_center(result):
    
    centers_x = []
    centers_y = []
    
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        centers_x.append(center_x)
        centers_y.append(center_y)

    centroid_x = np.mean(centers_x)
    centroid_y = np.mean(centers_y)

    return (centroid_x, centroid_y)

def find_coordinate(result, ara,  fov_h=70, fov_v=50, img_width=1280, img_height=720):
    if len(result.boxes) == 0:
        return None
    
    x, y = find_center(result)
    
    dlat, dlon, dalt = get_current_position(ara)
    
    offset_x = x - img_width / 2
    offset_y = y - img_height / 2
    
    angle_x = (offset_x / img_width) * fov_h
    angle_y = (offset_y / img_height) * fov_v
    
    distance_x = dalt * np.tan(np.radians(angle_x))
    distance_y = dalt * np.tan(np.radians(angle_y))
    
    delta_lat = (distance_y / 111111)
    delta_lon = (distance_x / (111111 * np.cos(np.radians(dlat))))
    
    person_lat = dlat + delta_lat
    person_lon = dlon + delta_lon
    person_alt = dalt
    
    return (person_lat, person_lon, person_alt)

x = 10
y = 10
person_detected = False
points = survey(ara, x=x, y=y)
point = 0

ara.arducopter_arm()
print("Arming motors")
ara.motors_armed_wait()
print("Motors armed")
ara.set_mode("GUIDED")
print("Setting mode to GUIDED")
ara.wait_heartbeat()
takeoff(ara, 10)
print("Going to takeoff")
survey(ara)

time.sleep(30)
send_waypoint(ara, *points[point])

while not person_detected:
    if point >= len(points):
        x *= 2
        y *= 2
        points = survey(ara, x=x, y=y)
        point = 0
        print(f"Expanding search area to x={x}, y={y}")
        
    current_position = get_current_position(ara)
    
    if is_at_waypoint(current_position, points[point]):
        print(f"Reached waypoint {point}: {points[point]}")
        point += 1
        if point < len(points):
            send_waypoint(ara, *points[point])
            
    img = window.screenshot()
    img = np.array(img)

    results = model(img)

    for result in results:
        for bbox in result.boxes:
            class_id = int(bbox.cls)
            label = model.names[class_id]
            x1, y1, x2, y2 = map(int, bbox.xyxy[0].tolist())

            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)


            cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if class_id == 0:
                person_coordinate = find_coordinate(result, ara)
                person_detected = True
                
    if person_detected:
        break
    
    time.sleep(1)
    
if person_detected:
    ara.set_mode("LOITER")
    print(f"Person coordinate at x:{person_coordinate[0]}, y:{person_coordinate[1]}, z:{person_coordinate[2]}")