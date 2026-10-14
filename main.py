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
LIDAR_PORT = os.getenv("LIDAR_PORT", "/dev/ttyUSB0")
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
    driver = LidarDriver(port=LIDAR_PORT, simulated=SIMULATE)
    driver.connect()

    analyzer = SectorAnalyzer(warning_threshold_m=1.2, emergency_threshold_m=0.35)

    print("[LIDAR-SERVICE] Pipeline di scansione attiva...")

    last_alert_time = 0.0

    for scan in driver.iter_scans():
        if not running:
            break

        # Analisi spaziale per settori
        analysis = analyzer.analyze_scan(scan)
        analysis["timestamp"] = time.time()

        now = time.time()
        # Se c'è un'anomalia/ostacolo, emetti evento di alert (limitato a max 5 Hz per non intasare)
        if analysis["has_alert"] and (now - last_alert_time > 0.2):
            last_alert_time = now
            alert_payload = {
                "timestamp": now,
                "theta": analysis.get("target_angle_deg", 0.0),
                "sector": analysis.get("target_sector", "FRONT"),
                "dist_m": analysis.get("min_distance_m", 1.0),
                "is_emergency": analysis["is_emergency"]
            }
            if mqtt_client and mqtt_client.is_connected():
                mqtt_client.publish("mar/lidar/alert", json.dumps(alert_payload))
            print(f"  🚨 [LIDAR ALERT] Ostacolo a {alert_payload['dist_m']}m in settore {alert_payload['sector']} (θ={alert_payload['theta']}°)")

        # Pubblica scan point cloud compresso (campionamento ogni 5 gradi per dashboard)
        downsampled = [(round(a, 1), round(d, 2)) for a, d in scan if int(a) % 4 == 0]
        radar_payload = {
            "timestamp": now,
            "points": downsampled,
            "sector_mins": analysis["sector_mins"]
        }

        if mqtt_client and mqtt_client.is_connected():
            mqtt_client.publish("mar/lidar/scan", json.dumps(radar_payload))


if __name__ == "__main__":
    main()
