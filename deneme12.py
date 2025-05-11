#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, math, argparse
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil

# ─── DİREK KOORDİNATLARI ────────────────────────────────────────────────
POLE1 = (-35.36165503, 149.16365917)
POLE2 = (-35.36355383, 149.16365917)
# ────────────────────────────────────────────────────────────────────────

def get_location_metres(orig, dNorth, dEast):
    R = 6378137.0
    dLat = dNorth / R
    dLon = dEast / (R * math.cos(math.radians(orig.lat)))
    return LocationGlobalRelative(
        orig.lat  + (dLat  * 180/math.pi),
        orig.lon  + (dLon  * 180/math.pi),
        orig.alt
    )

def add_circle(cmds, center, radius, alt, points=12, start_ang=0):
    """
    center etrafında tek tur: points kadar waypoint ekle.
    """
    for i in range(points):
        ang = math.radians(start_ang + 360.0 * i/points)
        dN  = radius * math.cos(ang)
        dE  = radius * math.sin(ang)
        wp  = get_location_metres(center, dN, dE)
        cmds.add(Command(
            0,0,0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,0, 0,0,0,0,
            wp.lat, wp.lon, alt
        ))

def upload_mission(vehicle):
    cmds = vehicle.commands
    cmds.clear()

    # ─── HOME NOKTASI & TAKEOFF ─────────────────────────────────────────
    home_alt    = 584.089966
    home        = LocationGlobalRelative(-35.362633, 149.165238, home_alt)
    takeoff_alt = 10.0

    # 0) HOME waypoint (zemin kotu)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        home.lat, home.lon, home.alt))

    # 1) TAKEOFF → takeoff_alt
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,
        home.lat, home.lon, takeoff_alt))

    # Hazır LocationGlobalRelative objeleri
    pole1 = LocationGlobalRelative(POLE1[0], POLE1[1], takeoff_alt)
    pole2 = LocationGlobalRelative(POLE2[0], POLE2[1], takeoff_alt)

    # ─── BİRİNCİ TUR ──────────────────────────────────────────────────────
    # 2) POLE1 etrafında çember
    add_circle(cmds, pole1, radius=30, alt=takeoff_alt, points=12, start_ang=0)
    # 3) POLE2’ye çapraz uçuş
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        pole2.lat, pole2.lon, takeoff_alt))
    # 4) POLE2 etrafında çember
    add_circle(cmds, pole2, radius=30, alt=takeoff_alt, points=12, start_ang=0)
    # 5) POLE1’e geri çapraz
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        pole1.lat, pole1.lon, takeoff_alt))

    # ─── İKİNCİ TUR ────────────────────────────────────────────────────────
    # Tekrar 2–5 adımlarını aynen uygula:
    add_circle(cmds, pole1, radius=30, alt=takeoff_alt, points=12, start_ang=0)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        pole2.lat, pole2.lon, takeoff_alt))
    add_circle(cmds, pole2, radius=30, alt=takeoff_alt, points=12, start_ang=0)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        pole1.lat, pole1.lon, takeoff_alt))

    # ─── İNİŞ ──────────────────────────────────────────────────────────────
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,0,0,0,0,0,
        home.lat, home.lon, 0))

    cmds.upload()
    print("Mission uploaded.")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', required=True,
                        help='örn. udp:127.0.0.1:14550')
    args = parser.parse_args()

    print("Connecting to", args.connect)
    vehicle = connect(args.connect, wait_ready=True)

    # Arming check devre dışı (simülasyonda)
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    upload_mission(vehicle)

    print("Arming & switching to AUTO")
    vehicle.mode  = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)
    vehicle.mode  = VehicleMode("AUTO")

    # Görev tamamlanana dek bekle
    while vehicle.mode.name == "AUTO":
        time.sleep(5)

    print("Mission complete, vehicle landing at home.")
    vehicle.close()

if __name__ == "__main__":
    main()
