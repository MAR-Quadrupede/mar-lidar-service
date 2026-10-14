#!/usr/bin/env python3
"""
M.A.R. — LiDAR Hardware Driver & Simulation Engine
Interfaces with RPLIDAR USB hardware or provides realistic simulated scans
when running in development environments without physical hardware attached.
"""

from typing import List, Tuple, Generator
import math
import random
import time

try:
    from rplidar import RPLidar
except ImportError:
    RPLidar = None


class LidarDriver:
    """Manages LiDAR hardware connection or falls back to simulation."""

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200, simulated: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.simulated = simulated or (RPLidar is None)
        self.lidar = None
        self.is_connected = False

    def connect(self) -> bool:
        """Connects to hardware or activates simulation."""
        if self.simulated:
            print("[LIDAR-DRIVER] 🧪 Inizializzazione driver in modalità SIMULATA.")
            self.is_connected = True
            return True

        if RPLidar is None:
            print("[LIDAR-DRIVER] ⚠️ rplidar package non trovato. Fallback su SIMULAZIONE.")
            self.simulated = True
            self.is_connected = True
            return True

        try:
            print(f"[LIDAR-DRIVER] Connessione a RPLiDAR su {self.port}...")
            self.lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=3)
            info = self.lidar.get_info()
            health = self.lidar.get_health()
            print(f"  ✓ LiDAR Connesso: Info={info}, Health={health}")
            self.is_connected = True
            return True
        except Exception as e:
            print(f"  ✗ Errore apertura LiDAR hardware ({e}). Attivazione SIMULATORE.")
            self.simulated = True
            self.is_connected = True
            return True

    def iter_scans(self) -> Generator[List[Tuple[float, float]], None, None]:
        """
        Yields scans: list of (angle_deg, distance_m).
        """
        if self.simulated:
            # Generates realistic synthetic 360-degree point cloud
            base_angle = 0.0
            simulated_obstacle_angle = 35.0
            while True:
                scan = []
                # Rotate simulated obstacle slightly over time
                simulated_obstacle_angle = (simulated_obstacle_angle + 0.5) % 360
                for a in range(0, 360, 2):
                    # Default room boundary around 3.5m with noise
                    dist = 3.5 + 0.2 * math.sin(math.radians(a * 4)) + random.uniform(-0.05, 0.05)

                    # Simulated obstacle at ~0.9m
                    diff = abs((a - simulated_obstacle_angle + 180) % 360 - 180)
                    if diff < 15:
                        dist = 0.85 + 0.05 * random.uniform(-1, 1)

                    scan.append((float(a), round(dist, 3)))
                yield scan
                time.sleep(0.1)  # 10 Hz simulated scan rate
        else:
            try:
                for scan_raw in self.lidar.iter_scans():
                    # scan_raw is list of (quality, angle_deg, distance_mm)
                    scan = []
                    for q, a, d_mm in scan_raw:
                        if q > 0 and d_mm > 0:
                            scan.append((float(a), d_mm / 1000.0))
                    yield scan
            except Exception as e:
                print(f"[LIDAR-DRIVER] Errore lettura stream: {e}")

    def stop(self):
        """Stops the motor and closes connection."""
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except Exception:
                pass
        self.is_connected = False
