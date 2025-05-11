#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# ─── DİREK KOORDİNATLARI ────────────────────────────────────────────────
POLE1 = (-35.36203338, 149.16497788)  # Direk 1 (lat, lon)
POLE2 = (-35.35940415, 149.16472942)  # Direk 2 (lat, lon)
# ────────────────────────────────────────────────────────────────────────

def get_location_metres(orig, dNorth, dEast):
    """orig’dan dNorth metre kuzeye, dEast metre doğuya ofsetli Location."""
    R = 6378137.0
    dLat = dNorth / R
    dLon = dEast / (R * math.cos(math.radians(orig.lat)))
    return LocationGlobalRelative(
        orig.lat  + (dLat  * 180/math.pi),
        orig.lon  + (dLon  * 180/math.pi),
        orig.alt
    )

def build_mission(home):
    """
    home: LocationGlobalRelative
    Returns list of mission items (dict) for upload_mission_int().
    """
    mission = []
    def add(cmd, frame, p1=0, p2=0, p3=0, p4=0,
            lat=0, lon=0, alt=0, autocont=1):
        mission.append({
            'command':      cmd,
            'frame':        frame,
            'param1':       p1,
            'param2':       p2,
            'param3':       p3,
            'param4':       p4,
            'latitude':     lat,
            'longitude':    lon,
            'altitude':     alt,
            'autocontinue': autocont
        })

    # 0) HOME’da NAV_WAYPOINT (zemin kotu)
    add(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        lat=home.lat, lon=home.lon, alt=home.alt)

    # 1) NAV_TAKEOFF → 10 m
    add(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        lat=home.lat, lon=home.lon, alt=10.0)

    # 2) CONDITION_CHANGE_ALT → 100 m’e tırman
    add(mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        p1=100.0, p2=5.0)

    # 3) “∞” deseni: POLE1→POLE2→POLE1→POLE2 etrafında 2 tur
    radius      = 50.0
    pattern_alt = 100.0
    steps       = 36
    for pole in (POLE1, POLE2, POLE1, POLE2):
        center = LocationGlobalRelative(pole[0], pole[1], pattern_alt)
        for i in range(steps + 1):
            ang = 360.0 * i / steps
            dN  = radius * math.cos(math.radians(ang))
            dE  = radius * math.sin(math.radians(ang))
            wp  = get_location_metres(center, dN, dE)
            add(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                lat=wp.lat, lon=wp.lon, alt=pattern_alt)

    # 4) NAV_LAND → tekrar home’da iniş
    add(mavutil.mavlink.MAV_CMD_NAV_LAND,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        lat=home.lat, lon=home.lon, alt=0)

    return mission

def upload_mission_int(vehicle, mission):
    """
    Upload via MAVLink2 MISSION_ITEM_INT.
    mission: list of dicts from build_mission().
    """
    master = vehicle._master
    sysid, compid = master.target_system, master.target_component
    count = len(mission)

    # 1) Görev sayısını gönder
    master.mav.mission_count_send(sysid, compid, count)

    # 2) Hem MISSION_REQUEST hem de MISSION_REQUEST_INT mesajlarına yanıt ver
    seq = 0
    while seq < count:
        msg = master.recv_match(
            type=['MISSION_REQUEST','MISSION_REQUEST_INT'],
            blocking=True, timeout=10
        )
        if not msg:
            raise RuntimeError("No MISSION_REQUEST(_INT) received")
        item = mission[msg.seq]
        master.mav.mission_item_int_send(
            sysid, compid,
            msg.seq,
            item['frame'],
            item['command'],
            0,                    # current
            item['autocontinue'],
            item['param1'],
            item['param2'],
            item['param3'],
            item['param4'],
            int(item['latitude']  * 1e7),
            int(item['longitude'] * 1e7),
            item['altitude']
        )
        seq += 1

    # 3) MISSION_ACK bekle
    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"Mission upload failed, ACK={ack.type}")
    print("Mission uploaded (MISSION_ITEM_INT).", flush=True)

def main():
    parser = argparse.ArgumentParser(
        description="Plane ∞+Pole mission (home→2 tur→home land)")
    parser.add_argument('--connect', required=True,
                        help='Bağlantı string’i, örn. udp:127.0.0.1:14550')
    args = parser.parse_args()

    print("Connecting to", args.connect, flush=True)
    vehicle = connect(args.connect, wait_ready=True)

    # SITL’de gyro vb. ön-arm hatalarını atla
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    # HOME konumu
    field_elev = 584.089966
    home = LocationGlobalRelative(-35.363262, 149.165238, field_elev)

    # Görevi hazırla ve yükle
    mission = build_mission(home)
    upload_mission_int(vehicle, mission)

    # Arm & AUTO modda başlat
    print("Arming and switching to AUTO", flush=True)
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
    while not vehicle.armed:
        time.sleep(0.5)
    vehicle.mode    = VehicleMode("AUTO")

    # Görev ve iniş tamamlanana dek bekle (~250s)
    time.sleep(250)

    print("Mission complete and landed.", flush=True)
    vehicle.close()

if __name__ == "__main__":
    main()
