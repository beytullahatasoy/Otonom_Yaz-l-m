#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, math, argparse
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil

# ------- GÖREV PARAMETRELERİ -------
# Pole koordinatlarını buraya gir:
POLE1 = (-35.36165503, 149.16365917)
POLE2 = (-35.36355383, 149.16365917)
# Yarım çember yarıçapı (metre):
RADIUS   = 50
# Kalkış irtifası (metre):
TAKEOFF_ALT = 30
# -----------------------------------

def get_location_metres(orig, dNorth, dEast):
    """
    orig:        LocationGlobalRelative
    dNorth,dEast: metre cinsinden offset
    return:      yeni LocationGlobalRelative
    """
    R = 6378137.0  # Dünya yarıçapı
    dLat = dNorth / R
    dLon = dEast  / (R * math.cos(math.radians(orig.lat)))
    return LocationGlobalRelative(
        orig.lat  + dLat * 180/math.pi,
        orig.lon  + dLon * 180/math.pi,
        orig.alt
    )

def add_half_circle(cmds, center, radius, alt, start_angle, direction, steps=20):
    """
    Yarım çember waypoint'leri ekler (180°).
      start_angle: derecelik başlangıç yönü (0=doğu, 90=kuzey, vb.)
      direction:   +1 ise CCW, -1 ise CW
      steps:       waypoint sayısı (daha yüksek → daha düzgün)
    """
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

    # 0) Home waypoint (düz uçuş irtifasında)
    home = vehicle.location.global_relative_frame
    cmds.add(Command(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        home.lat, home.lon, TAKEOFF_ALT
    ))

    # 1) Takeoff komutu
    cmds.add(Command(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,
        home.lat, home.lon, TAKEOFF_ALT
    ))

    # Orta nokta (polelar arası çapraz geçiş için)
    mid_lat = (POLE1[0] + POLE2[0]) / 2
    mid_lon = (POLE1[1] + POLE2[1]) / 2
    MID     = LocationGlobalRelative(mid_lat, mid_lon, TAKEOFF_ALT)

    # 2) 1. TUR:
    #   A) Pole1 CW yarım çember (270° → 90°)
    C1 = LocationGlobalRelative(POLE1[0], POLE1[1], TAKEOFF_ALT)
    add_half_circle(cmds, C1, radius=RADIUS, alt=TAKEOFF_ALT,
                    start_angle=270, direction=-1)
    #   B) Çapraz MID
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        MID.lat, MID.lon, MID.alt))
    #   C) Pole2 CCW yarım çember (90° → 270°)
    C2 = LocationGlobalRelative(POLE2[0], POLE2[1], TAKEOFF_ALT)
    add_half_circle(cmds, C2, radius=RADIUS, alt=TAKEOFF_ALT,
                    start_angle=90, direction=+1)
    #   D) Geri çapraz MID
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        MID.lat, MID.lon, MID.alt))

    # 3) 2. TUR (ters yönde):
    #   E) Pole1 CCW yarım çember (90° → 270°)
    add_half_circle(cmds, C1, radius=RADIUS, alt=TAKEOFF_ALT,
                    start_angle=90, direction=+1)
    #   F) Çapraz MID
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        MID.lat, MID.lon, MID.alt))
    #   G) Pole2 CW yarım çember (270° → 90°)
    add_half_circle(cmds, C2, radius=RADIUS, alt=TAKEOFF_ALT,
                    start_angle=270, direction=-1)

    # 4) Eve dönüş + iniş
    # → önce eve waypoint
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
        home.lat, home.lon, TAKEOFF_ALT))
    # → iniş
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,0,0,0,0,0,0,
        home.lat, home.lon, 0))

    cmds.upload()
    print("✅ Mission uploaded.")

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--connect', required=True,
                   help='örn. udp:127.0.0.1:14550')
    args = p.parse_args()

    print("🔗 Connecting to", args.connect)
    v = connect(args.connect, wait_ready=True)

    # Arming check atlama
    v.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    upload_mission(v)

    # 1) Waypoint listesini al
    v.commands.download()
    v.commands.wait_ready()

    # 2) Waypoint’ları numaralı şekilde yazdır
    print("📋 Mission waypoints:")
    for i, wp in enumerate(v.commands):
        print(f"{i}: CMD={wp.command} → lat={wp.x:.6f}, lon={wp.y:.6f}, alt={wp.z}")

    print("🔐 Arming & AUTO")
    v.mode  = VehicleMode("GUIDED")
    v.armed = True
    while not v.armed:
        time.sleep(0.5)
    v.mode = VehicleMode("AUTO")

    # Görev bitene kadar bekle
    while v.mode.name == "AUTO":
        time.sleep(2)

    print("✅ Mission complete. Landing/RTL.")
    v.close()

if __name__ == "__main__":
    main()
