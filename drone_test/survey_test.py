from pymavlink import mavutil
from ultralytics import YOLO
import pygetwindow as gw
import cv2
import numpy as np
import time

model = YOLO("yolov10n.pt")
ara = mavutil.mavlink_connection('udpin:10.1.91.94:14550')
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

def survey(ara, x=10, y=10, c=4):
    msg = ara.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    cx, cy, cz = msg.lat, msg.lon, msg.alt
    
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

def find_coordinate():
    pass

x = 10
y = 10
c = 4


ara.arducopter_arm()
print("Arming motors")
ara.motors_armed_wait()
print("Motors armed")
ara.set_mode("GUIDED")
print("Setting mode to GUIDED")
ara.wait_heartbeat()
takeoff(ara, 10)
print("Going to takeoff")
survey(ara, )

while True:
    img = window.screenshot()
    img = np.array(img)
    
    results = model(img)
    
    person_detected = False
    
    for result in results:
        for bbox in result.boxes:
            class_id = int(bbox.cls)
            label = model.names[class_id]
            
            x1, y1, x2, y2 = map(int, bbox.xyxy[0].tolist())
            
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            if class_id == 0:
                person_coordinate = find_coordinate()
                ara.set_mode("LOITER")
                person_detected = True
    
    if person_detected == True:
        break
    point = 0
    points = []
    if not points:
        points.extend(survey(ara, x=x, y=y, c=4))
        x *= 2
        y *= 2
        c *= 2
    if points[point] == (msg.lat, msg.lon, msg.alt, msg = ara.recv_match(type='GLOBAL_POSITION_INT', blocking=True)):
        send_waypoint(ara, *points[point])
        point += 1
    print(f"Going to location: lat={points[points[0]]}, lon={points[points[1]]}, alt={points[points[2]]}")