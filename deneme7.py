#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import argparse
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil

# ─── DİREK KOORDİNATLARI ────────────────────────────────────────────────
POLE1 = (-35.36235334, 149.16423544)  # Direk 1 (lat, lon)
POLE2 = (-35.36314926, 149.16423544)  # Direk 2 (lat, lon)
# ────────────────────────────────────────────────────────────────────────

def get_location_metres(orig, dNorth, dEast):
    """
    orig noktasından dNorth metre kuzeye, dEast metre doğuya ofsetli
    yeni LocationGlobalRelative noktasını hesaplar.
    """
    R    = 6378137.0
    dLat = dNorth / R
    dLon = dEast  / (R * math.cos(math.radians(orig.lat)))
    return LocationGlobalRelative(
        orig.lat  + (dLat * 180/math.pi),
        orig.lon  + (dLon * 180/math.pi),
        orig.alt
    )

def upload_mission(vehicle):
    # ——— AYARLANABİLİR PARAMETRELER ———
    home_lat       = -35.36251878   # pist başlangıç enlemi
    home_lon       = 149.16464667   # pist başlangıç boylamı
    field_elev     = 582.2          # zemin kotu (metre)
    takeoff_alt    = 10.0           # kalkış irtifası (metre)
    pole_alt       = 25.0           # direk waypoint irtifası (metre)
    pattern_alt    = 25.0           # ∞ deseni irtifası (metre)
    pattern_radius = 50             # ∞ deseni çember yarıçapı (metre)
    pattern_steps  = 12             # bir çemberdeki waypoint sayısı
    pattern_loops  = 2              # ∞ desenini kaç tur yapalım
    # ————————————————————————————————

    cmds = vehicle.commands
    cmds.clear()

    # Home noktası (kalkış/iniş)
    home = LocationGlobalRelative(home_lat, home_lon, field_elev)

    # 0) Zemin waypoint @ home
    cmds.add(Command(0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, 0, 0,
        home.lat, home.lon, home.alt))

    # 1) Takeoff → takeoff_alt
    cmds.add(Command(0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0,
        home.lat, home.lon, takeoff_alt))

    # 2) Direk 1 waypoint
    cmds.add(Command(0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, 0, 0,
        POLE1[0], POLE1[1], pole_alt))

    # 3) Direk 2 waypoint
    cmds.add(Command(0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, 0, 0,
        POLE2[0], POLE2[1], pole_alt))

    # 4) “∞” deseni: pattern_loops tur, çift çember döngüsü
    for _ in range(pattern_loops):
        for side in (-1, +1):  # önce batı (-1), sonra doğu (+1)
            center = get_location_metres(home, 0, side * pattern_radius)
            for i in range(pattern_steps + 1):
                ang = 360.0 * i / pattern_steps
                dN  = pattern_radius * math.cos(math.radians(ang))
                dE  = pattern_radius * math.sin(math.radians(ang)) * side
                wp  = get_location_metres(center, dN, dE)
                cmds.add(Command(0, 0, 0,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 0, 0, 0, 0,
                    wp.lat, wp.lon, pattern_alt))

    # 5) İniş @ home
    cmds.add(Command(0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0,
        home.lat, home.lon, 0))

    # 6) Görevleri yükle
    cmds.upload()
    print("Mission uploaded.", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="Sabit kanat görevi: pist → direkler → ∞ deseni → iniş"
    )
    parser.add_argument(
        '--connect', required=True,
        help='Bağlantı string’i, örn. udp:127.0.0.1:14550'
    )
    args = parser.parse_args()

    print(f"Bağlanıyor: {args.connect}", flush=True)
    vehicle = connect(args.connect, wait_ready=True)

    # SITL için sensör kontrollerini pas geç (gerçek uçuşta kaldırın)
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    # Görev profili yükle
    upload_mission(vehicle)

    # Arm et ve AUTO moda geçir
    print("Arm ediliyor ve AUTO moduna geçiliyor...", flush=True)
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
    while not vehicle.armed:
        time.sleep(0.5)
    vehicle.mode    = VehicleMode("AUTO")

    # Görev waypoint’lerini bitene kadar bekle
    print("Görev başladı, waypoint’leri izliyorum...", flush=True)
    total_wp = vehicle.commands.count
    while True:
        next_wp = vehicle.commands.next
        print(f" • Next waypoint: {next_wp}/{total_wp}", end='\r', flush=True)
        if next_wp >= total_wp:
            break
        time.sleep(1)

    # İnişi tamamlayana kadar bekle (disarm olana dek)
    print("\nİniş başladı, araç disarm olana kadar bekleniyor...", flush=True)
    while vehicle.armed:
        time.sleep(1)

    print("Görev tamamlandı, bağlantı kapanıyor.", flush=True)
    vehicle.close()

if __name__ == "__main__":
    main()
