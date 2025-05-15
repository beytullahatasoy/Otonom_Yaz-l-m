#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math
import argparse

from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from dronekit_sitl import SITL
from pymavlink import mavutil

# --------- GÖREV PARAMETRELERİ ----------
POLE1 = (-35.36165503, 149.16365917)
POLE2 = (-35.36355383, 149.16365917)
RADIUS    = 50      # metre
TAKEOFF_ALT = 30    # metre
# ---------------------------------------

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

    # home = SITL başlatırken verdiğimiz lokasyon
    home = vehicle.location.global_relative_frame

    # 0) home’da düz uçuş yüksekliğine çıkış (WAYPOINT)
    cmds.add(Command(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        home.lat, home.lon, TAKEOFF_ALT
    ))

    # 1) TAKEOFF komutu
    cmds.add(Command(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,
        home.lat, home.lon, TAKEOFF_ALT
    ))

    # orta nokta
    mid_lat = (POLE1[0] + POLE2[0]) / 2
    mid_lon = (POLE1[1] + POLE2[1]) / 2
    MID     = LocationGlobalRelative(mid_lat, mid_lon, TAKEOFF_ALT)

    # --- 1. TUR ---
    # A) Pole1 etrafında CW yarım çember
    C1 = LocationGlobalRelative(POLE1[0], POLE1[1], TAKEOFF_ALT)
    add_half_circle(cmds, C1, RADIUS, TAKEOFF_ALT,
                    start_angle=270, direction=-1)
    # B) çapraz geçiş
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        MID.lat, MID.lon, MID.alt))
    # C) Pole2 etrafında CCW yarım çember
    C2 = LocationGlobalRelative(POLE2[0], POLE2[1], TAKEOFF_ALT)
    add_half_circle(cmds, C2, RADIUS, TAKEOFF_ALT,
                    start_angle=90, direction=+1)
    # D) geri çapraz
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        MID.lat, MID.lon, MID.alt))

    # --- 2. TUR (ters yönde) ---
    # E) Pole1 CCW yarım çember
    add_half_circle(cmds, C1, RADIUS, TAKEOFF_ALT,
                    start_angle=90, direction=+1)
    # F) çapraz
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        MID.lat, MID.lon, MID.alt))
    # G) Pole2 CW yarım çember
    add_half_circle(cmds, C2, RADIUS, TAKEOFF_ALT,
                    start_angle=270, direction=-1)

    # --- Eve dönüş ve iniş ---
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        home.lat, home.lon, TAKEOFF_ALT))
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,0,0,0,0,0,
        home.lat, home.lon, 0))

    cmds.upload()
    print("✅ Mission uploaded.")

def arm_and_auto(vehicle):
    # GUIDED → ARM → throttle orta → AUTO
    vehicle.mode    = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(0.5)
    vehicle.armed   = True
    while not vehicle.armed:
        time.sleep(0.5)
    vehicle.channels.overrides['3'] = 1500
    time.sleep(1)
    vehicle.mode    = VehicleMode("AUTO")
    while vehicle.mode.name != "AUTO":
        time.sleep(0.5)

if __name__=="__main__":
    p = argparse.ArgumentParser()
    p.add_argument('--connect', default="127.0.0.1:14550",
                   help="UDP bağlantı (örn. 127.0.0.1:14550)")
    args = p.parse_args()

    # 1) SITL başlat (Plane + Harita pencereli)
    print("▶ Starting Plane SITL + map")
    sitl = SITL()
    sitl.launch([
        "--model","plane",
        "--console","--map",
        f"--home={POLE1[0]},{POLE1[1]},{TAKEOFF_ALT}"
    ], await_ready=True)

    # 2) DroneKit ile bağlan
    conn = f"udp:{args.connect}"
    print("🔗 Connecting to", conn)
    v = connect(conn, wait_ready=True)

    # 3) Arming check pasif
    v.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    # 4) Görevi oluştur & yükle
    upload_mission(v)

    # 5) Waypoint’leri kontrol et
    print("📋 Mission waypoints:")
    v.commands.download()
    v.commands.wait_ready()
    for i,wp in enumerate(v.commands):
        print(f" {i:>2}: CMD={wp.command}, lat={wp.x:.6f}, lon={wp.y:.6f}, alt={wp.z:.1f}")

    # 6) ARM & AUTO başlat
    print("🔐 Arming & AUTO")
    arm_and_auto(v)

    # 7) Görev tamamlanana kadar bekle
    print("⏳ Mission in progress, map on port 9002")
    while v.mode.name == "AUTO":
        time.sleep(2)

    print("✅ Mission complete. Disarming & stopping SITL.")
    v.armed = False
    v.close()
    sitl.stop()
