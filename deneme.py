from pymavlink import mavutil

#!/usr/bin/env python3
"""
figure8_autonomous_mission.py

Bu betik, DroneKit-Python kullanarak bir sabit kanat İHA ile 10.1.1’de tanimlanan yatay "8" (∞) manevrasını otonom olarak gerçekleştirir.

Özellikler:
- İki referans direk etrafinda dairesel turlarla şekillenen ∞ manevrasi
- Görev öncesi kalkiş (takeoff)
- Mission upload ve AUTO modunda başlatma
- Görev sonrasi RTL

Kullanim:
    pip install dronekit geographiclib
    python figure8_autonomous_mission.py --connect /dev/ttyAMA0

"""
import time,math
import argparse
from geographiclib.geodesic import Geodesic
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil

# --------- KULLANICI TARAFINDAN BELİRLENMESİ GEREKENLER ---------
POLE1 = (-35.36203338, 149.16497788)  # Direk 1 (lat, lon)
POLE2 = (-35.35940415, 149.16472942)  # Direk 2   
RADIUS = 30                     # Daire yarıçapı (metre)
NUM_POINTS = 36                 # Nokta sayısı
CRUISE_ALT = 100                # Görev uçuş yüksekliği (metre)
TAKEOFF_ALT = 10                # Kalkış sonrası yükseklik (metre)
# ------------------------------------------------------------------

def generate_circle(center, radius, points, altitude, start_bearing=90):
    geod = Geodesic.WGS84
    lat0, lon0 = center
    step = 360.0 / points
    waypoints = []
    for i in range(points + 1):
        bearing = start_bearing + i * step
        g = geod.Direct(lat0, lon0, bearing, radius)
        waypoints.append(LocationGlobalRelative(g['lat2'], g['lon2'], altitude))
    return waypoints


def clear_mission(vehicle):
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()


def upload_mission(vehicle):
    cmds = vehicle.commands
    cmds.clear()

    # 1) Takeoff wp: dummy LOCATION at POLE1
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,
                     0, 0, 0, 0,
                     POLE1[0], POLE1[1], TAKEOFF_ALT))

    # 2) Figure-eight pattern
    circle1 = generate_circle(POLE1, RADIUS, NUM_POINTS, CRUISE_ALT)
    circle2 = generate_circle(POLE2, RADIUS, NUM_POINTS, CRUISE_ALT)
    for wp in circle1 + circle2:
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                         0, 0, 0, 0,
                         wp.lat, wp.lon, wp.alt))

    # 3) RTL
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0,
                     0, 0, 0, 0,
                     0, 0, 0))

    cmds.upload()
    print("Mission uploaded.")


def arm_and_takeoff(vehicle, target_altitude):
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Bekleniyor: arma ediliyor...")
        time.sleep(1)

    print(f"Takeoff to {target_altitude}m")
    vehicle.simple_takeoff(target_altitude)
    # Altitude kontrolu
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Yükseklik: {alt:.1f}m")
        if alt >= target_altitude * 0.95:
            print("Hedef yüksekliğe ulaşıldı.")
            break
        time.sleep(1)


def main():
    parser = argparse.ArgumentParser(description='Figure-8 Autonomous Mission')
    parser.add_argument('--connect', help='Vehicle connection string', required=True)
    args = parser.parse_args()

    print(f"Bağlanılıyor: {args.connect}")
    vehicle = connect(args.connect, wait_ready=True)

    clear_mission(vehicle)
    upload_mission(vehicle)

    arm_and_takeoff(vehicle, TAKEOFF_ALT)

    print("Mission başlatılıyor...")
    vehicle.mode = VehicleMode("AUTO")

    # Görev izleme
    while True:
        next_wp = vehicle.commands.next
        print(f"Bir sonraki WP: {next_wp}")
        if next_wp == vehicle.commands.count:
            print("Son WP tamamlandı.")
            break
        time.sleep(2)

    # RTL moduna geçiş bekleniyor
    while not vehicle.mode.name == 'RTL':
        print("Görev bitişi, RTL moduna geçiliyor...")
        time.sleep(1)

    print("Görev tamamlandı, bağlantı kapatılıyor.")
    vehicle.close()


if __name__ == '__main__':
    main()

