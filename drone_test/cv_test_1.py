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

while True:
    img = window.screenshot()
    img = np.array(img)
    
    results = model(img)
    
    person_detcted = False
    
    for result in results:
        for bbox in result.boxes:
            class_id = int(bbox.cls)
            label = model.names[class_id]
            
            x1, y1, x2, y2 = map(int, bbox.xyxy[0].tolist())
            
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            if class_id == 0:
                person_detcted = True
    
    if person_detcted == True:
        
        ara.arducopter_arm()
        print("Arming motors")
        ara.motors_armed_wait()
        print("Motors armed")

        ara.set_mode("GUIDED")
        print("Setting mode to GUIDED")
        ara.wait_heartbeat()
        
        takeoff(ara, 10)
        print("Going to takeoff")
        
        time.sleep(60)
        break

landing()
print('Drone landed')
