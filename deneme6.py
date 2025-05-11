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
    home_lat       = -35.363262      # pist başlangıç enlem
    home_lon       = 149.165238      # pist başlangıç boylam
    field_elev     = 584.089966      # zemin kotu
    takeoff_alt    = 10.0            # kalkış irtifası (m)
    pole_alt       = 25.0            # direk waypoint irtifası (m)
    pattern_alt    = 25.0            # sonsuz deseni irtifası (m)
    pattern_radius = 50              # desendeki çember yarıçapı (m)
    pattern_steps  = 12              # bir çemberde nokta sayısı
    pattern_loops  = 2               # deseni kaç tur tekrarlayalım
    # ————————————————————————————

    cmds = vehicle.commands
    cmds.clear()

    # Home noktasını oluştur
    home = LocationGlobalRelative(home_lat, home_lon, field_elev)

    # 0) Zemin waypoint -home-
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        home.lat, home.lon, home.alt))

    # 1) Takeoff
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,
        home.lat, home.lon, takeoff_alt))

    # 2) Direk 1
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        POLE1[0], POLE1[1], pole_alt))

    # 3) Direk 2
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,0,0,0,0,0,
        POLE2[0], POLE2[1], pole_alt))

    # 4) “∞” deseni
    for _ in range(pattern_loops):
        for side in (-1, +1):
            center = get_location_metres(home, 0, side * pattern_radius)
            for i in range(pattern_steps + 1):
                ang = 360.0 * i / pattern_steps
                dN  = pattern_radius * math.cos(math.radians(ang))
                dE  = pattern_radius * math.sin(math.radians(ang)) * side
                wp  = get_location_metres(center, dN, dE)
                cmds.add(Command(0,0,0,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0,0,0,0,0,0,
                    wp.lat, wp.lon, pattern_alt))

    # 5) İniş -home-
    cmds.add(Command(0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,0,0,0,0,0,
        home.lat, home.lon, 0))

    # 6) Göreve yükle
    cmds.upload()
    print("Mission uploaded.", flush=True)


def main():
    # 1) Argümanları oku
    parser = argparse.ArgumentParser(
        description="Sabit kanat görevi: pist → direkler → ∞ deseni → iniş")
    
    parser.add_argument(
        '--connect', required=True,
        help='Bağlanti string’i, örn. udp:127.0.0.1:14550')
    
    args = parser.parse_args()

    # 2) Araca bağlan
    print(f"Bağlaniyor: {args.connect}", flush=True)
    vehicle = connect(args.connect, wait_ready=True)

    # 3) Arming check atla (SITL için)
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)

    # 4) Görevi upload et
    upload_mission(vehicle)

    # 5) Aracı arm et ve AUTO moduna geçir
    print("Arm ediliyor ve AUTO moduna geçiliyor...", flush=True)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)
    vehicle.mode = VehicleMode("AUTO")

    # 6) Görev tamamlanana dek bekle
    #    (burayı tahmini süreyle ya da durum kontrolüyle özelleştirebilirsin)
    mission_time = 200  # saniye cinsinden bekleme süresi
    print(f"Görev devam ediyor, yaklaşik {mission_time}s bekleniyor...", flush=True)
    time.sleep(mission_time)

    # 7) İşlem bitince bağlantıyı kapat
    print("Görev tamamlandi. Bağlanti kapatiliyor.", flush=True)
    vehicle.close()

if __name__ == "__main__":
    main()
