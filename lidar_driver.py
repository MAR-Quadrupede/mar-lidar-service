#!/usr/bin/env python3
"""
M.A.R. — LiDAR Hardware Driver & Auto-Discovery Engine
Supports native LD06 / LD19 / D300 / OKdo HAT LiDAR (0x54 protocol, 230400 baud),
RPLidar USB modules, and robust auto-reconnection across all Pi serial interfaces.
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
    """Manages LD06/OKdo HAT, RPLiDAR hardware connection with robust auto-probe."""

    def __init__(
        self,
        port: str = "/dev/ttyAMA0",
        baudrate: int = 230400,
        angle_offset_deg: float = 90.0,
        simulated: bool = False
    ):
        self.port = port
        self.baudrate = baudrate
        self.angle_offset_deg = angle_offset_deg
        self.simulated = simulated
        self.driver_type = "simulated" if simulated else "ld06"
        self.serial_conn: Optional[serial.Serial] = None
        self.rplidar_conn: Optional[RPLidar] = None
        self.is_connected = False

    @staticmethod
    def find_candidate_ports(preferred: str = "/dev/ttyAMA0", probe_usb: Optional[bool] = None) -> List[str]:
        """
        Porte candidate per il LiDAR.
        Di default SOLO le UART native del Raspberry (ttyAMA*/serial*/ttyS0) + la porta preferita:
        aprire a caso le porte USB (ttyUSB*/ttyACM*) resetta gli Arduino collegati (DTR toggle)
        e interrompe il flusso ultrasuoni/motion per secondi. Le USB si includono solo con
        LIDAR_PROBE_USB=1 o se LIDAR_PORT punta esplicitamente a una USB.
        """
        if probe_usb is None:
            probe_usb = os.getenv("LIDAR_PROBE_USB", "0").strip().lower() in ("1", "true", "yes")
        candidates: List[str] = []
        if preferred and os.path.exists(preferred):
            candidates.append(preferred)

        native = ["/dev/ttyAMA0", "/dev/serial0", "/dev/ttyAMA10", "/dev/ttyS0"]
        for kd in native:
            if os.path.exists(kd) and kd not in candidates:
                candidates.append(kd)
        for pattern in ("/dev/ttyAMA*", "/dev/serial*"):
            for p in sorted(glob.glob(pattern)):
                if p not in candidates:
                    candidates.append(p)

        if probe_usb:
            patterns = [
                "/dev/ttyUSB*", "/dev/ttyACM*",
                "/dev/cu.usb*", "/dev/cu.wchusb*", "/dev/cu.SLAB*", "/dev/cu.usbmodem*",
                "/dev/tty.usb*", "/dev/tty.wchusb*", "/dev/tty.SLAB*",
            ]
            for pattern in patterns:
                for p in sorted(glob.glob(pattern)):
                    if p not in candidates:
                        candidates.append(p)

        # /dev/serial0 è di solito un symlink a ttyAMA0: evita di sondare due volte lo stesso device
        seen_real = set()
        unique: List[str] = []
        for p in candidates:
            try:
                real = os.path.realpath(p)
            except Exception:
                real = p
            if real in seen_real:
                continue
            seen_real.add(real)
            unique.append(p)
        return unique

    def _probe_ld06_on_port(self, port_name: str, baud: int) -> bool:
        """Probes a serial port for valid LD06 0x54 packet header and length byte (0x2C)."""
        if serial is None or not os.path.exists(port_name):
            return False

        ser = None
        try:
            if getattr(self, "_connect_attempts", 1) <= 1:
                print(f"[LIDAR-DRIVER] Tentativo apertura LD06/OKdo HAT su {port_name} ({baud} baud)...")
            ser = serial.Serial(port_name, baud, timeout=0.04, exclusive=True)
            ser.reset_input_buffer()

            start_t = time.time()
            # Probe for up to 2.0s
            while time.time() - start_t < 2.0:
                if ser.in_waiting >= 47:
                    b_head = ser.read(1)
                    if b_head and b_head[0] == 0x54:
                        packet = ser.read(46)
                        if len(packet) == 46 and packet[0] == 0x2C:
                            # Confermato protocollo LD06: Header=0x54, VerLen=0x2C
                            start_angle = (((packet[4] << 8) | packet[3]) / 100.0) % 360.0
                            end_angle = (((packet[42] << 8) | packet[41]) / 100.0) % 360.0
                            if 0.0 <= start_angle <= 360.0 and 0.0 <= end_angle <= 360.0:
                                self.serial_conn = ser
                                self.port = port_name
                                self.baudrate = baud
                                self.driver_type = "ld06"
                                self.is_connected = True
                                print(f"  ✓ [LIDAR-DRIVER] Sensore LD06/OKdo HAT confermato e sincronizzato su {port_name} ({baud} baud)!")
                                return True
                else:
                    time.sleep(0.01)

            ser.close()
            return False
        except Exception as e:
            if ser:
                try:
                    ser.close()
                except Exception:
                    pass
            if "exclusive" in str(e).lower() or "busy" in str(e).lower() or "lock" in str(e).lower():
                print(f"  ⚠️ [LIDAR-DRIVER] {port_name} occupata da un altro processo: {e}")
            return False

    def connect(self) -> bool:
        """Attempts connection via LD06 raw protocol, then RPLidar, then retries or simulates."""
        # Se esplicitamente forzata la simulazione senza hardware
        if self.simulated:
            print("[LIDAR-DRIVER] 🧪 Inizializzazione driver in modalità SIMULATA (LIDAR_SIMULATE=true).")
            self.is_connected = True
            return True

        probe_usb = None
        if self.port and ("ttyUSB" in self.port or "ttyACM" in self.port or "/cu." in self.port):
            probe_usb = True
        ports_to_try = self.find_candidate_ports(self.port, probe_usb=probe_usb)
        self._connect_attempts = getattr(self, "_connect_attempts", 0) + 1
        if self._connect_attempts == 1 or self._connect_attempts % 10 == 0:
            print(f"[LIDAR-DRIVER] Scansione porte hardware candidate (tentativo {self._connect_attempts}): {ports_to_try}")

        # 1. Prova prima a 230400 baud (standard LD06 / OKdo HAT)
        for p in ports_to_try:
            if self._probe_ld06_on_port(p, 230400):
                return True

        # 2. Prova a 115200 baud (alcuni cloni LD06 / RPLidar)
        for p in ports_to_try:
            if self._probe_ld06_on_port(p, 115200):
                return True

        # 3. Prova RPLidar (solo su porte USB esplicite: il protocollo RPLidar invia comandi
        #    che un Arduino interpreterebbe come spazzatura)
        if RPLidar is not None and probe_usb:
            for p in ports_to_try:
                try:
                    print(f"[LIDAR-DRIVER] Tentativo apertura RPLiDAR su {p} (115200 baud)...")
                    lid = RPLidar(p, baudrate=115200, timeout=1)
                    info = lid.get_info()
                    self.rplidar_conn = lid
                    self.port = p
                    self.driver_type = "rplidar"
                    self.is_connected = True
                    print(f"  ✓ [LIDAR-DRIVER] RPLiDAR connesso su {p}: Info={info}")
                    return True
                except Exception:
                    pass

        # 4. Nessun sensore trovato
        if not self.simulated:
            print("  ⚠️ [LIDAR-DRIVER] Nessun pacchetto LiDAR hardware rilevato sui port seriali.")
            print("  ⏳ Rimango in modalità hardware: riproverò la sincronizzazione continua...")
            self.is_connected = False
            return False
        else:
            print("  🔧 Attivazione generatore di scansioni SIMULATE a 360°.")
            self.driver_type = "simulated"
            self.is_connected = True
            return True

    def iter_scans(self) -> Generator[List[Tuple[float, float]], None, None]:
        """
        Yields scans: list of (angle_deg, distance_m).
        Continually attempts hardware reconnect if port was not ready at startup.
        """
        while True:
            if self.simulated:
                yield from self._iter_simulated_scans()
                return

            if not self.is_connected or (self.serial_conn is None and self.rplidar_conn is None):
                # Ritenta connessione hardware con backoff (2 s → 15 s) per non martellare le porte
                if not self.connect():
                    attempts = getattr(self, "_connect_attempts", 1)
                    time.sleep(min(15.0, 2.0 + 1.5 * attempts))
                    continue
                self._connect_attempts = 0

            if self.driver_type == "ld06" and self.serial_conn:
                try:
                    yield from self._iter_ld06_scans()
                except Exception as e:
                    print(f"[LIDAR-DRIVER] Errore iterazione LD06 ({e}), riavvio probe...")
                    self.is_connected = False
                    if self.serial_conn:
                        try:
                            self.serial_conn.close()
                        except Exception:
                            pass
                        self.serial_conn = None
                    time.sleep(0.5)
            elif self.driver_type == "rplidar" and self.rplidar_conn:
                try:
                    yield from self._iter_rplidar_scans()
                except Exception as e:
                    print(f"[LIDAR-DRIVER] Errore iterazione RPLidar ({e}), riavvio probe...")
                    self.is_connected = False
                    self.rplidar_conn = None
                    time.sleep(0.5)
            else:
                time.sleep(0.5)

    def _iter_ld06_scans(self) -> Generator[List[Tuple[float, float]], None, None]:
        """Parses high-speed 0x54 stream from LD06 / LD19 / D300 LiDAR matching scratch_ref."""
        ser = self.serial_conn
        current_sweep: List[Tuple[float, float]] = []
        last_yield_time = time.time()
        SWEEP_INTERVAL_SEC = 0.10  # 10 Hz full scans

        while self.is_connected and ser and ser.is_open:
            try:
                # Evita accumulo backlog seriale
                if ser.in_waiting > 1500:
                    ser.reset_input_buffer()

                if ser.in_waiting < 47:
                    time.sleep(0.002)
                    continue

                byte_head = ser.read(1)
                if not byte_head or byte_head[0] != 0x54:
                    continue

                packet = ser.read(46)
                if len(packet) < 46 or packet[0] != 0x2C:
                    continue

                # Header ok: start_angle e end_angle Little Endian
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
                    # Applica offset fisico montaggio (default 90° per OKdo HAT frontale)
                    curr_angle = (point_angle + self.angle_offset_deg) % 360.0
                    dist_m = dist_mm / 1000.0

                    # Rileva ostacoli fisici a partire da 5 cm (0.05m) fino a 12 metri
                    if 0.05 <= dist_m <= 12.0:
                        current_sweep.append((round(curr_angle, 1), round(dist_m, 3)))

                now = time.time()
                if now - last_yield_time >= SWEEP_INTERVAL_SEC:
                    if len(current_sweep) >= 10:
                        yield current_sweep
                    current_sweep = []
                    last_yield_time = now

            except Exception as e:
                print(f"[LIDAR-DRIVER] Eccezione stream seriale LD06: {e}")
                raise e

    def _iter_rplidar_scans(self) -> Generator[List[Tuple[float, float]], None, None]:
        """Parses scans from RPLidar driver."""
        try:
            for scan_raw in self.rplidar_conn.iter_scans():
                scan = []
                for q, a, d_mm in scan_raw:
                    if q > 0 and 50 <= d_mm <= 12000:
                        adjusted_angle = (a + self.angle_offset_deg) % 360.0
                        scan.append((round(float(adjusted_angle), 1), round(d_mm / 1000.0, 3)))
                if len(scan) >= 10:
                    yield scan
        except Exception as e:
            print(f"[LIDAR-DRIVER] Errore stream RPLidar: {e}")
            raise e

    def _iter_simulated_scans(self) -> Generator[List[Tuple[float, float]], None, None]:
        """Generates realistic synthetic 360-degree point cloud only when explicitly configured."""
        while self.is_connected:
            scan = []
            for a in range(0, 360, 2):
                rad = math.radians(a)
                dist = 3.0 + 0.15 * math.sin(rad * 2) + random.uniform(-0.03, 0.03)
                if dist < 0.05:
                    continue
                scan.append((float(a), round(dist, 3)))
            yield scan
            time.sleep(0.1)

    def stop(self):
        """Stops hardware and frees serial port."""
        self.is_connected = False
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None
        if self.rplidar_conn:
            try:
                self.rplidar_conn.stop()
                self.rplidar_conn.stop_motor()
                self.rplidar_conn.disconnect()
            except Exception:
                pass
            self.rplidar_conn = None
