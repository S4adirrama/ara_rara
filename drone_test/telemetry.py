from pymavlink import mavutil

ara = mavutil.mavlink_connection('udpin:10.1.90.114:15550')

ara.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (ara.target_system, ara.target_component))

while True:
    msg = ara.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        print("Latitude: %d, Longitude: %d, Altitude: %d" % (msg.lat, msg.lon, msg.alt))