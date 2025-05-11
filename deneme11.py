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
    R = 6378137.0
    dLat = dNorth / R
    dLon = dEast / (R * math.cos(math.radians(orig.lat)))
    return LocationGlobalRelative(
        orig.lat  + dLat * 180/math.pi,
        orig.lon  + dLon * 180/math.pi,
        orig.alt
    )

def add_circle(cmds, center, radius, alt, points=12, start_ang=0):
    for i in range(points + 1):
        for i in range(points):
         ang = math.radians(start_ang + 360.0 * i/points)
         dN =  radius * math.cos(ang)
         dE =  radius * math.sin(ang)
         wp = get_location_metres(center, dN, dE)
         cmds.add(Command(
             0,0,0,
             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
             0,0,0,0,0,0,
             wp.lat, wp.lon, alt
         ))


def upload_mission(vehicle):
    cmds = vehicle.commands
    cmds.clear()

    # 1) home noktası
    field_elev = 584.089966
    home = LocationGlobalRelative(-35.362633, 149.165238, field_elev)

    # --- 0: NAV_WAYPOINT (home, alt=field_elev) ---
    cmds.add(Command(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        home.lat, home.lon, home.alt
    ))

    # --- 1: NAV_TAKEOFF to 10m above home ---
    takeoff_alt = 10.0
    cmds.add(Command(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,
        home.lat, home.lon, takeoff_alt
    ))

    # --- 2: pole1 etrafında tek tur ---
    center1 = LocationGlobalRelative(POLE1[0], POLE1[1], takeoff_alt)
    add_circle(cmds, center1, radius=30, alt=takeoff_alt, points=12, start_ang=0)

    # --- 3: çapraz geçiş: pole1 güneybatı → pole2 kuzeydoğu ---
    cross1 = LocationGlobalRelative(
        (POLE1[0]+POLE2[0])/2 - 0.0001,  # azıcık batıya
        (POLE1[1]+POLE2[1])/2 + 0.0001,  # azıcık kuzeye
        takeoff_alt
    )
    cmds.add(Command(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        cross1.lat, cross1.lon, cross1.alt
    ))

    # --- 4: pole2 etrafında tek tur ---
    center2 = LocationGlobalRelative(POLE2[0], POLE2[1], takeoff_alt)
    add_circle(cmds, center2, radius=30, alt=takeoff_alt, points=12, start_ang=0)

    # --- 5: geriye çapraz geçiş: pole2 kuzeybatı → pole1 güneydoğu ---
    cross2 = LocationGlobalRelative(
        (POLE1[0]+POLE2[0])/2 + 0.0001,
        (POLE1[1]+POLE2[1])/2 - 0.0001,
        takeoff_alt
    )
    cmds.add(Command(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        cross2.lat, cross2.lon, cross2.alt
    ))

    # --- 6: LAND at home ---
    cmds.add(Command(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,0,0,0,0,0,
        home.lat, home.lon, 0
    ))

    cmds.upload()
    print("Mission uploaded.")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', required=True,
                        help='e.g. udp:127.0.0.1:14550')
    args = parser.parse_args()

    print("Connecting to", args.connect)
    vehicle = connect(args.connect, wait_ready=True)

    # arming check kapat
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    upload_mission(vehicle)

    print("Arming & AUTO")
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
    while not vehicle.armed:
        time.sleep(0.5)
    vehicle.mode    = VehicleMode("AUTO")

    # Tamamlanana kadar bekle (yaklaşık 5-6 dakika sürer)
    while vehicle.mode.name == "AUTO":
        time.sleep(5)

    print("Script uçuşu tamamladı, araç şimdi iniyor/RTL modunda olabilir.")
    vehicle.close()

if __name__ == "__main__":
    main()
