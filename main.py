#!/usr/bin/env python3
"""
M.A.R. — mar-lidar-service microservice daemon
Streams 360-degree point clouds, evaluates sector proximity, triggers saccades,
and publishes scan data to the MQTT message bus.
"""

import os
import sys
import time
import json
import signal

from lidar_driver import LidarDriver
from sector_analyzer import SectorAnalyzer
from slam_engine import LidarSlamEngine

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
LIDAR_PORT = os.getenv("LIDAR_PORT", "/dev/ttyAMA0")
LIDAR_BAUD = int(os.getenv("LIDAR_BAUD", "230400"))
LIDAR_OFFSET_DEG = float(os.getenv("LIDAR_OFFSET_DEG", "0.0"))
SIMULATE = os.getenv("LIDAR_SIMULATE", "false").lower() in ("true", "1", "yes")

driver = None
running = True


def signal_handler(sig, frame):
    global running
    print("\n[LIDAR-SERVICE] Chiusura del servizio LiDAR...")
    running = False
    if driver:
        driver.stop()
    sys.exit(0)


def main():
    global driver, running
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 60)
    print(" 🐾 M.A.R. — PERIMETRAL LIDAR & PROXIMITY TRIGGER SERVICE")
    print("=" * 60)

    # 1. Connessione MQTT
    mqtt_client = None
    if mqtt:
        try:
            mqtt_client = mqtt.Client(client_id="mar-lidar-service")
            mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            mqtt_client.loop_start()
            print(f"  ✓ Connesso a MQTT Broker ({MQTT_BROKER}:{MQTT_PORT})")
        except Exception as e:
            print(f"  ⚠️ MQTT non disponibile ({e}). Esecuzione standalone.")

    # 2. Inizializzazione Hardware / Simulatore
    driver = LidarDriver(
        port=LIDAR_PORT,
        baudrate=LIDAR_BAUD,
        angle_offset_deg=LIDAR_OFFSET_DEG,
        simulated=SIMULATE
    )
    driver.connect()

    analyzer = SectorAnalyzer(warning_threshold_m=1.2, emergency_threshold_m=0.35)
    slam = LidarSlamEngine(grid_resolution_m=0.05)

    print("[LIDAR-SERVICE] Pipeline di scansione e motore SLAM 2D attivi...")

    last_alert_time = 0.0
    last_alert_state = False
    last_map_pub_time = 0.0
    last_slam_log_time = 0.0

    for scan in driver.iter_scans():
        if not running:
            break

        now = time.time()

        # 1. Aggiornamento continuo SLAM: stima posa (X, Y, Theta) e mappatura globale della stanza
        slam_res = slam.update_scan(scan, now=now)

        # 2. Analisi spaziale per settori
        analysis = analyzer.analyze_scan(scan)
        analysis["timestamp"] = now

        # Payload completo con tutti gli 8 settori per la sensor fusion
        alert_payload = {
            "timestamp": now,
            "has_alert": analysis["has_alert"],
            "is_emergency": analysis["is_emergency"],
            "theta": analysis.get("target_angle_deg", 0.0),
            "sector": analysis.get("target_sector", "FRONT"),
            "sector_en": analysis.get("target_sector_en", "FRONT"),
            "dist_m": analysis.get("min_distance_m", 99.0),
            "front_distance_m": analysis.get("front_distance_m", 99.0),
            "sector_mins": analysis.get("sector_mins", {}),
            "sector_mins_en": analysis.get("sector_mins_en", {}),
            "pose": {
                "x_m": slam_res["x_m"],
                "y_m": slam_res["y_m"],
                "theta_deg": slam_res["theta_deg"]
            }
        }

        # Pubblica alert a 5 Hz (o istantaneo se transizione di allerta)
        if (analysis["has_alert"] != last_alert_state) or (now - last_alert_time > 0.2):
            last_alert_time = now
            last_alert_state = analysis["has_alert"]
            if mqtt_client and mqtt_client.is_connected():
                mqtt_client.publish("mar/lidar/alert", json.dumps(alert_payload))
                mqtt_client.publish("mar/lidar/telemetry", json.dumps(alert_payload))
            if analysis["has_alert"]:
                print(f"  🚨 [LIDAR ALERT] Ostacolo a {alert_payload['dist_m']}m in settore {alert_payload['sector']} (θ={alert_payload['theta']}°)")

        # Pubblica posa SLAM ad alta frequenza (~8 Hz)
        if mqtt_client and mqtt_client.is_connected():
            mqtt_client.publish("mar/slam/pose", json.dumps(slam_res))

        # Pubblica mappa globale con nuvola di punti della stanza a 2 Hz
        if (now - last_map_pub_time >= 0.5) and (mqtt_client and mqtt_client.is_connected()):
            last_map_pub_time = now
            map_payload = slam.get_full_map_payload()
            mqtt_client.publish("mar/slam/map", json.dumps(map_payload))

        # Log periodico localizzazione spaziale ogni 3 secondi
        if now - last_slam_log_time >= 3.0:
            last_slam_log_time = now
            bounds = slam_res["room_bounds_m"]
            print(
                f"  📍 [SLAM LOCALIZZAZIONE] X={slam_res['x_m']:+0.2f}m Y={slam_res['y_m']:+0.2f}m θ={slam_res['theta_deg']}° | "
                f"Mappa: {slam_res['map_points_count']} punti (Stanza: {bounds['width_m']:.1f}m x {bounds['length_m']:.1f}m, Area: {bounds['area_m2']:.1f}m²)"
            )

        # Point cloud densa: 1° bin (mediana) — più precisione, meno rumore multi-echo
        bins: dict = {}
        for a, d in scan:
            if not (0.12 <= d <= 12.0):
                continue
            key = int(round(a)) % 360
            bins.setdefault(key, []).append(d)
        downsampled = []
        for ang in sorted(bins.keys()):
            vals = bins[ang]
            # 20° percentile: cattura ostacolo vicino senza il singolo outlier
            dist = float(sorted(vals)[max(0, len(vals) // 5)])
            downsampled.append((float(ang), round(dist, 3)))
        radar_payload = {
            "timestamp": now,
            "points": downsampled,
            "point_count": len(scan),
            "cloud_count": len(downsampled),
            "sector_mins": analysis["sector_mins"],
            "front_distance_m": analysis.get("front_distance_m"),
            "slam": {
                "x_m": slam_res["x_m"],
                "y_m": slam_res["y_m"],
                "theta_deg": slam_res["theta_deg"],
                "dist_traveled_m": slam_res["distance_traveled_m"]
            }
        }

        if mqtt_client and mqtt_client.is_connected():
            mqtt_client.publish("mar/lidar/scan", json.dumps(radar_payload))


if __name__ == "__main__":
    main()
