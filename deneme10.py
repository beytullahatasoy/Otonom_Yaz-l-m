#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# deneme10.py

import time, math, argparse
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil

# Direk koordinatları
DIREK1 = (-35.36095507, 149.16298831)
DIREK2 = (-35.36454118, 149.16409056)

# Ayarlar
TAKEOFF_ALT = 20.0       # metre
CIRCLE_ALT   = 50.0      # metre
RADIUS       = 30        # metre
CIRCLE_STEPS = 16        # daireyi böleceğimiz nokta sayısı

def get_location_metres(orig, dNorth, dEast):
    R = 6378137.0
    dLat = dNorth / R
    dLon = dEast / (R * math.cos(math.radians(orig.lat)))
    return LocationGlobalRelative(
        orig.lat  + dLat  * 180/math.pi,
        orig.lon  + dLon  * 180/math.pi,
        orig.alt
    )

def upload_mission(vehicle):
    cmds = vehicle.commands
    cmds.clear()

    # HOME
    home = LocationGlobalRelative(vehicle.location.global_frame.lat,
                                  vehicle.location.global_frame.lon,
                                  0)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        home.lat, home.lon, 0))

    # TAKEOFF
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,
        home.lat, home.lon, TAKEOFF_ALT))

    # 1. Daire (Direk1)
    center1 = LocationGlobalRelative(DIREK1[0], DIREK1[1], CIRCLE_ALT)
    for i in range(CIRCLE_STEPS+1):
        ang = 2*math.pi * i / CIRCLE_STEPS
        dN = RADIUS * math.cos(ang)
        dE = RADIUS * math.sin(ang)
        wp = get_location_metres(center1, dN, dE)
        cmds.add(Command(0,0,0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,0,0,0,0,0,
            wp.lat, wp.lon, CIRCLE_ALT))

    # Çapraz geçiş (ortası)
    mid12 = LocationGlobalRelative(
        (DIREK1[0]+DIREK2[0])/2,
        (DIREK1[1]+DIREK2[1])/2,
        CIRCLE_ALT
    )
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        mid12.lat, mid12.lon, CIRCLE_ALT))

    # 2. Daire (Direk2)
    center2 = LocationGlobalRelative(DIREK2[0], DIREK2[1], CIRCLE_ALT)
    for i in range(CIRCLE_STEPS+1):
        ang = 2*math.pi * i / CIRCLE_STEPS
        dN = RADIUS * math.cos(ang)
        dE = RADIUS * math.sin(ang)
        wp = get_location_metres(center2, dN, dE)
        cmds.add(Command(0,0,0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,0,0,0,0,0,
            wp.lat, wp.lon, CIRCLE_ALT))

    # Geri çapraz
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        mid12.lat, mid12.lon, CIRCLE_ALT))

    # İniş (HOME)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,0,0,0,0,0,
        home.lat, home.lon, 0))

    cmds.upload()
    print(f"✅ Görev yüklendi ({len(cmds)} waypoint).")
    time.sleep(1)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', required=True,
                        help='örn. udp:127.0.0.1:14550')
    args = parser.parse_args()

    print("Bağlanılıyor:", args.connect)
    vehicle = connect(args.connect, wait_ready=True)
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    upload_mission(vehicle)

    print("Arming & AUTO moda geçiliyor…")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)
    vehicle.mode = VehicleMode("AUTO")

    # Yaklaşık görev süresi kadar bekle
    time.sleep(300)
    print("Görev bitti, sistemi kapatıyorum.")
    vehicle.close()

if __name__ == "__main__":
    main()
#-35.36095507 149.16298831, -35.36454118 149.16409056 