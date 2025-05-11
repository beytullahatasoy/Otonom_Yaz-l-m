#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, math, argparse
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil

def upload_mission(vehicle):
    cmds = vehicle.commands
    cmds.clear()

    # 1) Sabit HOME konumu (senin file’daki 584.089966m yükseklik)
    field_elev = 584.089966
    home = LocationGlobalRelative(-35.363262, 149.165238, field_elev)

    # --- QGC satır 0: NAV_WAYPOINT at home, altitude=field_elev ---
    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0,
        0, 0, 0, 0,
        home.lat, home.lon, home.alt
    ))

    # --- QGC satır 1: NAV_TAKEOFF to 10m above home ---
    takeoff_alt = 10.0
    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0,
        0, 0, 0, 0,
        home.lat, home.lon, takeoff_alt
    ))

    # 2) “∞” deseni: iki 50 m yarıçaplı daire, hepsini 100 m irtifada uçuruyoruz
    radius = 50
    pattern_alt = 100.0
    steps = 12
    for side in (-1, +1):
        # dairenin merkezi
        center = get_location_metres(home, 0, side * radius)
        for i in range(steps + 1):
            ang = 360.0 * i / steps
            dN = radius * math.cos(math.radians(ang))
            dE = radius * math.sin(math.radians(ang)) * side
            wp = get_location_metres(center, dN, dE)
            cmds.add(Command(
                0,0,0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,0,
                0,0,0,0,
                wp.lat, wp.lon, pattern_alt
            ))

    # 3) RTL
    cmds.add(Command(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,0,0,0,0,0,
        0,0,0
    ))

    cmds.upload()
    print("Mission uploaded.")

def get_location_metres(orig, dNorth, dEast):
    R = 6378137.0
    dLat = dNorth / R
    dLon = dEast / (R * math.cos(math.radians(orig.lat)))
    return LocationGlobalRelative(
        orig.lat + dLat * 180/math.pi,
        orig.lon + dLon * 180/math.pi,
        orig.alt
    )

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', required=True,
                        help='e.g. udp:127.0.0.1:14550')
    args = parser.parse_args()

    print("Connecting to", args.connect)
    vehicle = connect(args.connect, wait_ready=True)
    # gerekirse gyro hatalarını atla
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    upload_mission(vehicle)

    print("Arming and switching to AUTO")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)
    vehicle.mode = VehicleMode("AUTO")

    # Görev boyunca bekle
    time.sleep(180)

    print("Done, returning to launch")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(20)
    vehicle.close()

if __name__ == "__main__":
    main()
