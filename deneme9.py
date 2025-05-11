#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, math, argparse
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil

POLE1 = (-35.36235334, 149.16423544)
POLE2 = (-35.36314926, 149.16423544)

def get_location_metres(orig, dNorth, dEast):
    R = 6378137.0
    dLat = dNorth / R
    dLon = dEast / (R * math.cos(math.radians(orig.lat)))
    return LocationGlobalRelative(
        orig.lat  + (dLat * 180/math.pi),
        orig.lon  + (dLon * 180/math.pi),
        orig.alt
    )

def upload_first_loop(vehicle):
    cmds = vehicle.commands
    cmds.clear()

    # Ayarlar
    home     = LocationGlobalRelative(-35.36251878, 149.16464667, 582.2)
    takeoff  = 10.0
    pattern  = 25.0
    radius   = 50
    steps    = 36

    # 0) HOME (ground waypoint)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        home.lat, home.lon, home.alt))

    # 1) TAKEOFF
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,
        home.lat, home.lon, takeoff))

    # Merkezler
    c1 = LocationGlobalRelative(POLE1[0], POLE1[1], pattern)
    c2 = LocationGlobalRelative(POLE2[0], POLE2[1], pattern)

    # 2) İlk sonsuz tur (mavi):
    #   a) Pole1 CCW
    for i in range(steps+1):
        ang = 360.0 * i / steps
        dN  = radius * math.cos(math.radians(ang))
        dE  = radius * math.sin(math.radians(ang))
        wp  = get_location_metres(c1, dN, dE)
        cmds.add(Command(0,0,0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,0,0,0,0,0,
            wp.lat, wp.lon, pattern))

    #   b) Çapraz geçiş → Pole2
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        c2.lat, c2.lon, pattern))

    #   c) Pole2 CW
    for i in range(steps+1):
        ang = 360.0 * (steps - i) / steps
        dN  = radius * math.cos(math.radians(ang))
        dE  = radius * math.sin(math.radians(ang))
        wp  = get_location_metres(c2, dN, dE)
        cmds.add(Command(0,0,0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,0,0,0,0,0,
            wp.lat, wp.lon, pattern))

    # 3) İniş
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,0,0,0,0,0,
        home.lat, home.lon, 0))

    cmds.upload()
    print("First loop mission uploaded.", flush=True)

def main():
    parser = argparse.ArgumentParser(description="Sadece ilk 8 turunu uç")
    parser.add_argument('--connect', required=True,
        help='Bağlantı string’i (örn. udp:127.0.0.1:14550)')
    args = parser.parse_args()

    vehicle = connect(args.connect, wait_ready=True)
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    upload_first_loop(vehicle)

    vehicle.mode  = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)
    vehicle.mode  = VehicleMode("AUTO")

    # Bitene kadar bekle
    total = vehicle.commands.count
    while vehicle.commands.next < total:
        time.sleep(0.5)
    while vehicle.armed:
        time.sleep(0.5)

    print("İlk tur tamamlandı.")
    vehicle.close()

if __name__ == "__main__":
    main()
