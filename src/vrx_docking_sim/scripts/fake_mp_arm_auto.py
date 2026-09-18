#!/usr/bin/env python3
# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""Dev/validation tool, hors run-time -- pas installé comme entry point.
Run on the host after fake_mission_planner.py: `python3 fake_mp_arm_auto.py`

Arm and switch to AUTO purely via an independent MAVLink client (the same
Mission-Planner-facing port), to confirm the whole "Mission Planner drives
the mission" path end-to-end without any ROS/MAVROS involvement.

Targets system 1 / component 1 explicitly (the vehicle's known autopilot
address) rather than relying on wait_heartbeat()'s auto-detected source --
mavlink-router relays every endpoint's traffic to every other endpoint, so
this port also sees MAVROS's own heartbeat (a different component on the
same system), and auto-detection is not reliable when multiple senders are
present on the link.
"""
import time

from pymavlink import mavutil

conn = mavutil.mavlink_connection('udp:0.0.0.0:14550')

print('Waiting for the *autopilot* heartbeat (component 1) specifically...')
deadline = time.time() + 10
target_sys = None
while time.time() < deadline:
    msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if msg is None:
        continue
    print(f'  saw heartbeat: sys={msg.get_srcSystem()} comp={msg.get_srcComponent()} '
          f'type={msg.type} autopilot={msg.autopilot}')
    if msg.get_srcComponent() == 1:  # MAV_COMP_ID_AUTOPILOT1
        target_sys = msg.get_srcSystem()
        break

if target_sys is None:
    print('Never saw the autopilot heartbeat (component 1)')
    raise SystemExit(1)

target_comp = 1
print(f'Targeting system {target_sys} component {target_comp}')

conn.mav.command_long_send(
    target_sys, target_comp,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
    1, 0, 0, 0, 0, 0, 0)
ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
print(f'Arm ACK: {ack}')

conn.mav.set_mode_send(
    target_sys,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    10)  # Rover AUTO
time.sleep(1)

while True:
    hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if hb is None:
        print('no more heartbeats')
        break
    if hb.get_srcComponent() == 1:
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f'Autopilot heartbeat: custom_mode={hb.custom_mode} armed={armed}')
        break
