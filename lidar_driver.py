#!/usr/bin/env python3
"""
M.A.R. — LiDAR Hardware Driver & Simulation Engine
Supports native LD06 / LD19 / D300 / OKdo HAT LiDAR (0x54 protocol, 230400 baud),
RPLidar USB modules, and realistic simulated 360-degree scans for testing.
"""

from typing import List, Tuple, Generator, Optional
import math
import random
import time
import os
import glob

try:
    import serial
except ImportError:
    serial = None

try:
    from rplidar import RPLidar
except ImportError:
    RPLidar = None


class LidarDriver:
    """Manages LD06/OKdo HAT, RPLiDAR hardware connection or falls back to simulation."""

    def __init__(
        self,
        port: str = "/dev/ttyAMA0",
        baudrate: int = 230400,
        angle_offset_deg: float = 180.0,
        simulated: bool = False
    ):
        self.port = port
        self.baudrate = baudrate
        self.angle_offset_deg = angle_offset_deg
        self.simulated = simulated
        self.driver_type = "simulated" if simulated else "auto"
        self.serial_conn: Optional[serial.Serial] = None
        self.rplidar_conn: Optional[RPLidar] = None
        self.is_connected = False

    @staticmethod
    def find_candidate_ports() -> List[str]:
        """Finds potential serial/UART ports on Raspberry Pi or development OS."""
        candidates = []
        patterns = [
            "/dev/ttyAMA*", "/dev/ttyACM*", "/dev/ttyUSB*", "/dev/serial0",
            "/dev/cu.usb*", "/dev/cu.wchusb*", "/dev/cu.SLAB*", "/dev/cu.usbmodem*",
            "/dev/tty.usb*", "/dev/tty.wchusb*", "/dev/tty.SLAB*"
        ]
        for pattern in patterns:
            candidates.extend(glob.glob(pattern))
        return sorted(list(set(candidates)))

    def connect(self) -> bool:
        """Attempts connection via LD06 raw protocol, then RPLidar, then simulation."""
        if self.simulated:
            print("[LIDAR-DRIVER] 🧪 Inizializzazione driver in modalità SIMULATA.")
            self.is_connected = True
            return True

        # Test candidate ports (start with specified port, then auto-discovered)
        ports_to_try = [self.port]
        for p in self.find_candidate_ports():
            if p not in ports_to_try:
                ports_to_try.append(p)

        # 1. Try LD06 / OKdo HAT LiDAR at 230400 baud
        if serial is not None:
            for p in ports_to_try:
                try:
                    print(f"[LIDAR-DRIVER] Tentativo apertura LD06/OKdo HAT su {p} ({self.baudrate} baud)...")
                    ser = serial.Serial(p, self.baudrate, timeout=0.5)
                    ser.reset_input_buffer()

                    # Test reading for 0x54 header
                    start_t = time.time()
                    detected = False
                    while time.time() - start_t < 1.0:
                        b = ser.read(1)
                        if b and b[0] == 0x54:
                            packet = ser.read(46)
                            if len(packet) == 46:
                                detected = True
                                break

                    if detected:
                        self.serial_conn = ser
                        self.port = p
                        self.driver_type = "ld06"
                        self.is_connected = True
                        print(f"  ✓ [LIDAR-DRIVER] LD06 / OKdo HAT LiDAR rilevato e sincronizzato su {p}!")
                        return True
                    else:
                        ser.close()
                except Exception:
                    pass

        # 2. Try RPLidar USB at 115200 baud
        if RPLidar is not None:
            for p in ports_to_try:
                try:
                    print(f"[LIDAR-DRIVER] Tentativo apertura RPLiDAR su {p} (115200 baud)...")
                    lid = RPLidar(p, baudrate=115200, timeout=2)
                    info = lid.get_info()
                    self.rplidar_conn = lid
                    self.port = p
                    self.driver_type = "rplidar"
                    self.is_connected = True
                    print(f"  ✓ [LIDAR-DRIVER] RPLiDAR connesso su {p}: Info={info}")
                    return True
                except Exception:
                    pass

        # 3. Fallback to Simulation
        print("  ⚠️ Nessun sensore LiDAR hardware rilevato sui port seriali.")
        print("  🔧 Attivazione generatore di scansioni SIMULATE a 360° per test.")
        self.simulated = True
        self.driver_type = "simulated"
        self.is_connected = True
        return True

    def iter_scans(self) -> Generator[List[Tuple[float, float]], None, None]:
        """
        Yields scans: list of (angle_deg, distance_m).
        """
        if self.driver_type == "ld06" and self.serial_conn:
            yield from self._iter_ld06_scans()
        elif self.driver_type == "rplidar" and self.rplidar_conn:
            yield from self._iter_rplidar_scans()
        else:
            yield from self._iter_simulated_scans()

    def _iter_ld06_scans(self) -> Generator[List[Tuple[float, float]], None, None]:
        """Parses high-speed 0x54 stream from LD06 / LD19 / D300 LiDAR."""
        ser = self.serial_conn
        current_sweep: List[Tuple[float, float]] = []
        last_yield_time = time.time()
        SWEEP_INTERVAL_SEC = 0.10  # 10 Hz full scans

        while self.is_connected:
            try:
                # Evita overflow senza scartare pacchetti frequenti (buffer seriale tipico 4096B)
                if ser.in_waiting > 3000:
                    ser.reset_input_buffer()

                byte_head = ser.read(1)
                if not byte_head or byte_head[0] != 0x54:
                    continue

                packet = ser.read(46)
                if len(packet) < 46:
                    continue

                start_angle = (((packet[4] << 8) | packet[3]) / 100.0) % 360.0
                end_angle = (((packet[42] << 8) | packet[41]) / 100.0) % 360.0
                diff = end_angle - start_angle
                if diff < 0:
                    diff += 360.0

                angle_step = diff / 11.0 if (0.1 <= diff <= 60.0) else 0.8

                for i in range(12):
                    base_idx = 5 + (i * 3)
                    dist_mm = (packet[base_idx + 1] << 8) | packet[base_idx]
                    intensity = packet[base_idx + 2]

                    point_angle = (start_angle + (i * angle_step)) % 360.0
                    # Applica offset fisico montaggio (default 180° per allineamento frontale)
                    curr_angle = (point_angle + self.angle_offset_deg) % 360.0
                    dist_m = dist_mm / 1000.0

                    # Filtro anti-rumore carrozzeria / supporti (< 5cm)
                    if 0.05 <= dist_m <= 12.0:
                        current_sweep.append((curr_angle, round(dist_m, 3)))

                now = time.time()
                if now - last_yield_time >= SWEEP_INTERVAL_SEC:
                    if len(current_sweep) >= 15:
                        yield current_sweep
                    current_sweep = []
                    last_yield_time = now

            except Exception as e:
                print(f"[LIDAR-DRIVER] Errore stream seriale LD06: {e}")
                time.sleep(0.1)

    def _iter_rplidar_scans(self) -> Generator[List[Tuple[float, float]], None, None]:
        """Parses scans from RPLidar driver."""
        try:
            for scan_raw in self.rplidar_conn.iter_scans():
                scan = []
                for q, a, d_mm in scan_raw:
                    # Filtra riflessi interni corpo robot / cavi / staffe (< 50mm)
                    if q > 0 and 50 <= d_mm <= 12000:
                        adjusted_angle = (a + self.angle_offset_deg) % 360.0
                        scan.append((float(adjusted_angle), round(d_mm / 1000.0, 3)))
                if len(scan) >= 15:
                    yield scan
        except Exception as e:
            print(f"[LIDAR-DRIVER] Errore stream RPLidar: {e}")

    def _iter_simulated_scans(self) -> Generator[List[Tuple[float, float]], None, None]:
        """Generates realistic synthetic 360-degree point cloud for headless test."""
        while self.is_connected:
            scan = []
            for a in range(0, 360, 2):
                # Corridoio/stanza realistica: fronte aperto a ~3.0m, pareti laterali a 2.5m, retro a ~1.8m
                rad = math.radians(a)
                # Modello rettangolare stanza (6m x 5m)
                # Fronte a 0° (+X): ~3.0m
                dx = 3.0 / max(0.2, math.cos(rad)) if -45 <= a <= 45 or a >= 315 else 99.0
                dist = 3.0 + 0.15 * math.sin(rad * 2) + random.uniform(-0.03, 0.03)
                if dist < 0.22:
                    continue
                scan.append((float(a), round(dist, 3)))
            yield scan
            time.sleep(0.1)  # 10 Hz rate

    def stop(self):
        """Stops hardware and frees serial port."""
        self.is_connected = False
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        if self.rplidar_conn:
            try:
                self.rplidar_conn.stop()
                self.rplidar_conn.stop_motor()
                self.rplidar_conn.disconnect()
            except Exception:
                pass
