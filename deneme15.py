#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, math, argparse
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil
import dronekit_sitl

# ------- GÖREV PARAMETRELERİ -------
POLE1 = (-35.36165503, 149.16365917)
POLE2 = (-35.36355383, 149.16365917)
RADIUS   = 50
TAKEOFF_ALT = 30
# -----------------------------------

def get_location_metres(orig, dNorth, dEast):
    R = 6378137.0
    dLat = dNorth / R
    dLon = dEast  / (R * math.cos(math.radians(orig.lat)))
    return LocationGlobalRelative(
        orig.lat  + dLat  * 180/math.pi,
        orig.lon  + dLon  * 180/math.pi,
        orig.alt
    )

def add_half_circle(cmds, center, radius, alt, start_angle, direction, steps=20):
    span = 180 * direction
    for i in range(steps+1):
        ang = math.radians(start_angle + span * (i/steps))
        dN  = radius * math.cos(ang)
        dE  = radius * math.sin(ang)
        wp  = get_location_metres(center, dN, dE)
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

    home = vehicle.location.global_relative_frame
    # 0) Home direkt waypoint (hızı sabit tutmak için ALT’da)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        home.lat, home.lon, TAKEOFF_ALT))
    # 1) Takeoff
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,
        home.lat, home.lon, TAKEOFF_ALT))

    mid_lat = (POLE1[0] + POLE2[0]) / 2
    mid_lon = (POLE1[1] + POLE2[1]) / 2
    MID     = LocationGlobalRelative(mid_lat, mid_lon, TAKEOFF_ALT)

    # 2) 1. TUR
    C1 = LocationGlobalRelative(*POLE1, TAKEOFF_ALT)
    add_half_circle(cmds, C1, RADIUS, TAKEOFF_ALT, start_angle=270, direction=-1)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        MID.lat, MID.lon, MID.alt))
    C2 = LocationGlobalRelative(*POLE2, TAKEOFF_ALT)
    add_half_circle(cmds, C2, RADIUS, TAKEOFF_ALT, start_angle=90, direction=+1)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        MID.lat, MID.lon, MID.alt))

    # 3) 2. TUR (ters yönde)
    add_half_circle(cmds, C1, RADIUS, TAKEOFF_ALT, start_angle=90, direction=+1)
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        MID.lat, MID.lon, MID.alt))
    add_half_circle(cmds, C2, RADIUS, TAKEOFF_ALT, start_angle=270, direction=-1)

    # 4) Eve dönüş + iniş
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        home.lat, home.lon, TAKEOFF_ALT))
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,0,0,0,0,0,0,
        home.lat, home.lon, 0))

    cmds.upload()
    print("✅ Mission uploaded.")

def main():
    # 1) SITL’i başlat (alt= argümanını kaldırdık!)
    sitl = dronekit_sitl.start_default(model='ardupilot', home='-35.362633,149.165238')
    conn = sitl.connection_string()
    print("🔗 SITL başlatıldı:", conn)

    # 2) Bağlan ve parametre ayarla
    vehicle = connect(conn, wait_ready=True)
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    # 3) Görevi yükle
    upload_mission(vehicle)

    # 4) Listeyi indir & yazdır (isteğe bağlı)
    vehicle.commands.download()
    vehicle.commands.wait_ready()
    for i, wp in enumerate(vehicle.commands):
        print(f"{i}: CMD={wp.command}, lat={wp.x:.6f}, lon={wp.y:.6f}, alt={wp.z:.1f}")

    # 5) Kalkış ve AUTO
    vehicle.mode = VehicleMode("GUIDED");  vehicle.armed = True
    while not vehicle.armed: time.sleep(0.5)
    vehicle.mode = VehicleMode("AUTO")

    # 6) Let the mission run
    while vehicle.mode.name == "AUTO":
        time.sleep(1)

    print("✅ Uçuş tamamlandı, iniş/Roaming.")
    vehicle.close()
    sitl.stop()

if __name__ == "__main__":
    main()