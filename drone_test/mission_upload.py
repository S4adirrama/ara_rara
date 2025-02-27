from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

ara = mavutil.mavlink_connection('udpin:10.1.90.114:15550')

ara.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (ara.target_system, ara.target_component))

waypoints = [
    mavlink2.MAVLink_mission_item_message(
        ara.target_system, ara.target_component,
        0,
        mavlink2.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavlink2.MAV_CMD_NAV_WAYPOINT,
        0, 1, 0, 0, 0, 0,
        -35.363261, 149.165230, 20
    ),
    mavlink2.MAVLink_mission_item_message(
        ara.target_system, ara.target_component,
        1,
        mavlink2.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavlink2.MAV_CMD_NAV_WAYPOINT,
        0, 1, 0, 0, 0, 0,
        -35.363000, 149.165000, 25
    )
]

ara.mav.mission_count_send(
    ara.target_system, ara.target_component,
    len(waypoints)
)

for i, waypoint in enumerate(waypoints):
    ara.mav.send(waypoint)

msg = ara.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK'], blocking=True)
if msg.get_type() == 'MISSION_ACK':
    print("Mission successfully uploaded")
else:
    print(f"Requesting item {msg.seq}")
