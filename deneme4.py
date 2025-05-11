#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import argparse
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil

# ─── DİREK KOORDİNATLARI ────────────────────────────────────────────────
POLE1 = (-35.36203338, 149.16497788)  # Direk 1 (lat, lon)
POLE2 = (-35.35940415, 149.16472942)  # Direk 2 (lat, lon)
# ────────────────────────────────────────────────────────────────────────

def get_location_metres(orig, dNorth, dEast):
    """orig’dan dNorth metre kuzeye, dEast metre doğuya ofsetli yeni Location yarat."""
    R = 6378137.0
    dLat = dNorth / R
    dLon = dEast / (R * math.cos(math.radians(orig.lat)))
    return LocationGlobalRelative(
        orig.lat  + (dLat  * 180/math.pi),
        orig.lon  + (dLon  * 180/math.pi),
        orig.alt
    )

def upload_mission(vehicle):
    cmds = vehicle.commands
    cmds.clear()

    # ─── HOME NOKTASI ────────────────────────────────────────────────────
    field_elev = 584.089966
    home = LocationGlobalRelative(-35.363262, 149.165238, field_elev)
    # ─────────────────────────────────────────────────────────────────────

    # 0) Home’da NAV_WAYPOINT (zemin kotu)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0, 0,0,0,0,
        home.lat, home.lon, home.alt))

    # 1) NAV_TAKEOFF → 10m
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0, 0,0,0,0,
        home.lat, home.lon, 10.0))

    # 2) Direk 1’e NAV_WAYPOINT (100m)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0, 0,0,0,0,
        POLE1[0], POLE1[1], 100.0))

    # 3) Direk 2’ye NAV_WAYPOINT (100m)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0, 0,0,0,0,
        POLE2[0], POLE2[1], 100.0))

    # 4) “∞” deseni: iki 50m yarıçaplı daire, 100m’de
    radius     = 50
    pattern_alt = 100.0
    steps      = 12
    for side in (-1, +1):
        center = get_location_metres(home, 0, side * radius)
        for i in range(steps + 1):
            ang = 360.0 * i / steps
            dN  = radius * math.cos(math.radians(ang))
            dE  = radius * math.sin(math.radians(ang)) * side
            wp  = get_location_metres(center, dN, dE)
            cmds.add(Command(0,0,0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,0, 0,0,0,0,
                wp.lat, wp.lon, pattern_alt))

    # 5) NAV_LAND → home noktasında iniş
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,0, 0,0,0,0,
        home.lat, home.lon, 0))

    cmds.upload()
    print("Mission uploaded.", flush=True)

def main():
    parser = argparse.ArgumentParser(description="Plane ∞ + Direk mission (AUTO-only)")
    parser.add_argument('--connect', required=True,
                        help='Bağlantı string’i, örn. udp:127.0.0.1:14550')
    args = parser.parse_args()

    print("Connecting to", args.connect, flush=True)
    vehicle = connect(args.connect, wait_ready=True)

    # SITL’de gyro vs. hatalarını atlamak için
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    upload_mission(vehicle)

    print("Arming & switching to AUTO", flush=True)
    vehicle.mode  = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)
    vehicle.mode  = VehicleMode("AUTO")

    # Görev ve iniş tamamlanana dek bekle (yaklaşık 200s)
    time.sleep(200)

    print("Mission complete and landed. Closing connection.", flush=True)
    vehicle.close()

if __name__ == "__main__":
    main()
