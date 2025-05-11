#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, math, argparse
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil

# ------- BURAYA KENDİ DİREK KOORDİNATLARINI YAZ -------
POLE1 = (-35.36165503, 149.16365917)
POLE2 = (-35.36355383, 149.16365917)
# ------------------------------------------------------

def get_location_metres(orig, dNorth, dEast):
    """Metre cinsinden ofseti global lokasyona çevirir."""
    R = 6378137.0
    dLat = dNorth / R
    dLon = dEast  / (R * math.cos(math.radians(orig.lat)))
    return LocationGlobalRelative(
        orig.lat + (dLat * 180/math.pi),
        orig.lon + (dLon * 180/math.pi),
        orig.alt
    )

def add_half_circle(cmds, center, radius, alt, start_angle, direction, steps=20):
    """
    Yarım çember oluşturur (180°).
      direction = +1 ise CCW, -1 ise CW.
    """
    span = 180 * direction
    for i in range(steps + 1):
        angle = math.radians(start_angle + span * (i/steps))
        dN = radius * math.cos(angle)
        dE = radius * math.sin(angle)
        wp = get_location_metres(center, dN, dE)
        cmds.add(Command(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0,
            wp.lat, wp.lon, alt
        ))

def upload_mission(vehicle):
    cmds = vehicle.commands
    cmds.clear()

    # Home (başlangıç/iniş) noktası
    HOME = LocationGlobalRelative(-35.362633, 149.165238, 584.089966)

    # 0) Home waypoint
    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, 0, 0,
        HOME.lat, HOME.lon, HOME.alt
    ))

    # 1) Takeoff 30m
    TAKEOFF_ALT = 30
    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0,
        HOME.lat, HOME.lon, TAKEOFF_ALT
    ))

    # Orta nokta (direkler arası)
    mid_lat = (POLE1[0] + POLE2[0]) / 2
    mid_lon = (POLE1[1] + POLE2[1]) / 2
    MID = LocationGlobalRelative(mid_lat, mid_lon, TAKEOFF_ALT)

    # 2) 1. TUR: Pole1 CW yarım çember → MID → Pole2 CCW yarım çember → MID
    C1 = LocationGlobalRelative(POLE1[0], POLE1[1], TAKEOFF_ALT)
    add_half_circle(cmds, C1, radius=50, alt=TAKEOFF_ALT,
                    start_angle=270, direction=-1)
    cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        MID.lat, MID.lon, MID.alt))
    C2 = LocationGlobalRelative(POLE2[0], POLE2[1], TAKEOFF_ALT)
    add_half_circle(cmds, C2, radius=50, alt=TAKEOFF_ALT,
                    start_angle=90, direction=+1)
    cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        MID.lat, MID.lon, MID.alt))

    # 3) 2. TUR: Pole1 CCW yarım çember → MID → Pole2 CW yarım çember
    add_half_circle(cmds, C1, radius=50, alt=TAKEOFF_ALT,
                    start_angle=90, direction=+1)
    cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        MID.lat, MID.lon, MID.alt))
    add_half_circle(cmds, C2, radius=50, alt=TAKEOFF_ALT,
                    start_angle=270, direction=-1)

    # 4) Eve dönüş ve iniş
    cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        HOME.lat, HOME.lon, TAKEOFF_ALT))
    cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,0,0,0,0,0,0,
        HOME.lat, HOME.lon, 0))

    cmds.upload()
    print("🚀 Mission uploaded.")

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--connect', required=True,
                   help='e.g. udp:127.0.0.1:14550')
    args = p.parse_args()

    print("🔗 Connecting to", args.connect)
    v = connect(args.connect, wait_ready=True)
    v.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    upload_mission(v)

    print("🔐 Arming & AUTO")
    v.mode  = VehicleMode("GUIDED")
    v.armed = True
    while not v.armed:
        time.sleep(0.5)
    v.mode = VehicleMode("AUTO")

    # Bitişini bekle
    while v.mode.name == "AUTO":
        time.sleep(2)

    print("✅ Done.")
    v.close()

if __name__ == "__main__":
    main()
