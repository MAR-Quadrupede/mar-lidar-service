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

    print("[LIDAR-SERVICE] Pipeline di scansione attiva...")

    last_alert_time = 0.0
    last_alert_state = False

    for scan in driver.iter_scans():
        if not running:
            break

        # Analisi spaziale per settori
        analysis = analyzer.analyze_scan(scan)
        analysis["timestamp"] = time.time()

        now = time.time()
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
            "sector_mins_en": analysis.get("sector_mins_en", {})
        }

        # Pubblica a 5 Hz (o istantaneo se transizione di allerta)
        if (analysis["has_alert"] != last_alert_state) or (now - last_alert_time > 0.2):
            last_alert_time = now
            last_alert_state = analysis["has_alert"]
            if mqtt_client and mqtt_client.is_connected():
                mqtt_client.publish("mar/lidar/alert", json.dumps(alert_payload))
                mqtt_client.publish("mar/lidar/telemetry", json.dumps(alert_payload))
            if analysis["has_alert"]:
                print(f"  🚨 [LIDAR ALERT] Ostacolo a {alert_payload['dist_m']}m in settore {alert_payload['sector']} (θ={alert_payload['theta']}°)")

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
        }

        if mqtt_client and mqtt_client.is_connected():
            mqtt_client.publish("mar/lidar/scan", json.dumps(radar_payload))


if __name__ == "__main__":
    main()
