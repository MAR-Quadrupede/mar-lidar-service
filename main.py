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
import threading
import numpy as np

from lidar_driver import LidarDriver
from sector_analyzer import SectorAnalyzer
from slam_engine import LidarSlamEngine

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, (np.floating, np.float32, np.float64)):
            return float(obj)
        if isinstance(obj, (np.integer, np.int32, np.int64)):
            return int(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)

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
IMU_YAW_MAX_AGE_S = float(os.getenv("LIDAR_IMU_MAX_AGE_S", "0.5"))

# Ultimo yaw IMU (gradi, antiorario positivo) ricevuto da mar/imu/telemetry → prior di rotazione SLAM
_imu_state = {"yaw_deg": None, "ts": 0.0}


def _on_mqtt_connect(client, userdata, flags, rc, properties=None):
    code = getattr(rc, "value", rc)
    if code == 0:
        client.subscribe("mar/imu/telemetry")
        print(f"  ✓ Connesso a MQTT Broker ({MQTT_BROKER}:{MQTT_PORT}) — mar/imu/telemetry sottoscritto")
    else:
        print(f"  ⚠️ MQTT connect rifiutato (rc={rc})")


def _on_mqtt_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode("utf-8", "ignore"))
    except Exception:
        return
    if msg.topic == "mar/imu/telemetry" and isinstance(data, dict):
        yaw = data.get("yaw")
        if yaw is None:
            return
        try:
            _imu_state["yaw_deg"] = float(yaw)
            _imu_state["ts"] = time.time()
        except (TypeError, ValueError):
            pass


def _imu_yaw_for_slam(now: float):
    if _imu_state["yaw_deg"] is None or (now - _imu_state["ts"]) > IMU_YAW_MAX_AGE_S:
        return None
    return _imu_state["yaw_deg"]

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
        # connect_async + loop_start → retry infinito (DNS del cluster lento al boot) e
        # re-subscribe in on_connect ad ogni riconnessione.
        try:
            mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="mar-lidar-service")
        except Exception:
            mqtt_client = mqtt.Client(client_id="mar-lidar-service")
        mqtt_client.on_connect = _on_mqtt_connect
        mqtt_client.on_message = _on_mqtt_message
        mqtt_client.reconnect_delay_set(min_delay=1, max_delay=10)
        mqtt_client.connect_async(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
        print(f"  ✓ MQTT loop avviato verso {MQTT_BROKER}:{MQTT_PORT} (retry automatico)")

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
    status_state = {"last_pub": 0.0}

    def publish_status(now: float, scanning: bool, scan_hz: float = 0.0):
        if not (mqtt_client and mqtt_client.is_connected()):
            return
        status_state["last_pub"] = now
        mqtt_client.publish("mar/lidar/status", json.dumps({
            "timestamp": now,
            "connected": bool(driver.is_connected and scanning),
            "driver": driver.driver_type,
            "port": driver.port,
            "baud": driver.baudrate,
            "scan_hz": round(scan_hz, 1),
            "slam_points": slam.get_status()["map_points_count"],
            "imu_prior": _imu_yaw_for_slam(now) is not None,
            "rx_bytes": int(getattr(driver, "last_rx_bytes", 0) or 0),
            "probe": getattr(driver, "last_probe_summary", "") or "",
        }))

    # Stato "offline" finché il sensore non parla (la dashboard distingue LiDAR assente da silenzio MQTT)
    def status_heartbeat():
        while running:
            now = time.time()
            if now - status_state["last_pub"] > 2.0:
                publish_status(now, scanning=False)
            time.sleep(1.0)

    threading.Thread(target=status_heartbeat, daemon=True).start()

    scan_times: list = []

    for scan in driver.iter_scans():
        if not running:
            break

        now = time.time()
        scan_times.append(now)
        scan_times[:] = [t for t in scan_times if now - t <= 2.0]

        # 1. Aggiornamento continuo SLAM: posa (X, Y, Theta) con prior IMU di yaw se disponibile
        slam_res = slam.update_scan(scan, imu_yaw_deg=_imu_yaw_for_slam(now), now=now)

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
            "point_count": len(scan),
            "sector_mins": analysis.get("sector_mins", {}),
            "sector_mins_en": analysis.get("sector_mins_en", {}),
            "pose": {
                "x_m": slam_res["x_m"],
                "y_m": slam_res["y_m"],
                "theta_deg": slam_res["theta_deg"]
            }
        }

        # Invio immediato ad alta reattività a MQTT (zero lag verso Decision Engine e Gateway)
        if mqtt_client and mqtt_client.is_connected():
            mqtt_client.publish("mar/lidar/alert", json.dumps(alert_payload, cls=NumpyEncoder))
            mqtt_client.publish("mar/lidar/telemetry", json.dumps(alert_payload, cls=NumpyEncoder))
            mqtt_client.publish("mar/slam/pose", json.dumps(slam_res, cls=NumpyEncoder))

        if now - status_state["last_pub"] >= 1.0:
            publish_status(now, scanning=True, scan_hz=len(scan_times) / 2.0)

        if analysis["has_alert"] and (analysis["has_alert"] != last_alert_state or now - last_alert_time > 1.0):
            last_alert_time = now
            last_alert_state = analysis["has_alert"]
            print(f"  🚨 [LIDAR ALERT] Ostacolo a {alert_payload['dist_m']}m in settore {alert_payload['sector']} (θ={alert_payload['theta']}°)")
        elif not analysis["has_alert"]:
            last_alert_state = False

        # Pubblica mappa globale con nuvola di punti della stanza a ~2 Hz
        if (now - last_map_pub_time >= 0.4) and (mqtt_client and mqtt_client.is_connected()):
            last_map_pub_time = now
            map_payload = slam.get_full_map_payload()
            mqtt_client.publish("mar/slam/map", json.dumps(map_payload, cls=NumpyEncoder))

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
        # Unico payload mar/lidar/scan per scansione (prima veniva pubblicato due volte)
        radar_payload = {
            "timestamp": now,
            "points": downsampled,
            "point_count": len(scan),
            "cloud_count": len(downsampled),
            "sector_mins": analysis["sector_mins"],
            "sector_mins_en": analysis.get("sector_mins_en", {}),
            "front_distance_m": analysis.get("front_distance_m"),
            "slam": {
                "x_m": slam_res["x_m"],
                "y_m": slam_res["y_m"],
                "theta_deg": slam_res["theta_deg"],
                "dist_traveled_m": slam_res["distance_traveled_m"],
                "icp_ok": slam_res.get("icp_ok", True),
            }
        }

        if mqtt_client and mqtt_client.is_connected():
            mqtt_client.publish("mar/lidar/scan", json.dumps(radar_payload, cls=NumpyEncoder))


if __name__ == "__main__":
    main()
