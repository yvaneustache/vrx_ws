#!/usr/bin/env python3
# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""Dev/validation tool, hors run-time (comme scripts/plot_apf_field.py de
vrx_apf_dock) -- pas installé comme entry point.

Stand-in for Mission Planner: an independent MAVLink client on the
Mission-Planner-facing mavlink-router output (UDP 14550), talking directly
to ArduPilot -- no ROS/MAVROS involved at all, unlike an earlier (wrong)
livrable-6 test that used /mavros/mission/push. Run on the host (not inside
the container), after mavlink-router and ArduPilot SITL are up:

  python3 fake_mission_planner.py
"""
import sys
import time

from pymavlink import mavutil

print('Connecting (listening) on udp:0.0.0.0:14550 ...')
conn = mavutil.mavlink_connection('udp:0.0.0.0:14550')

print('Waiting for heartbeat from vehicle...')
hb = conn.wait_heartbeat(timeout=10)
if hb is None:
    print('NO HEARTBEAT RECEIVED')
    sys.exit(1)
print(f'Heartbeat from system {conn.target_system} component {conn.target_component}')

waypoints = [
    # (lat, lon, alt) -- offsets from wherever the vehicle currently is,
    # values don't matter much here, we're validating the MISSION protocol
    # transaction itself works end-to-end through mavlink-router.
    (-35.3670000, 149.1640000, 0),
    (-35.3670000, 149.1650000, 0),
]

print(f'Uploading {len(waypoints)}-waypoint mission via MISSION protocol...')
conn.mav.mission_count_send(conn.target_system, conn.target_component, len(waypoints), 0)

uploaded = 0
deadline = time.time() + 10
while uploaded < len(waypoints) and time.time() < deadline:
    msg = conn.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=3)
    if msg is None:
        continue
    seq = msg.seq
    lat, lon, alt = waypoints[seq]
    conn.mav.mission_item_int_send(
        conn.target_system, conn.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1,
        0, 2, 0, 0,
        int(lat * 1e7), int(lon * 1e7), alt)
    uploaded += 1
    print(f'  sent waypoint {seq}')

ack = conn.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
if ack is None:
    print('NO MISSION_ACK RECEIVED')
    sys.exit(1)
print(f'MISSION_ACK type={ack.type} (0 = MAV_MISSION_ACCEPTED)')

if ack.type != 0:
    print('MISSION UPLOAD REJECTED')
    sys.exit(1)

print('Mission uploaded successfully via the Mission-Planner-facing port, independent of ROS/MAVROS.')
