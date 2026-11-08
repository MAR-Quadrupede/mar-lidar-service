#!/usr/bin/env python3
"""
M.A.R. — LiDAR Sector Analyzer & Proximity Trigger
Segments 360-degree LiDAR point clouds into 8 operational sectors (Fronte, Front-DX, etc.).
The distance of a sector is the near confirmed return (~10th percentile), not the median:
a sheet or a hand that covers only part of the 45° wedge must move the number.
EMA smoothing still rejects a single speckle, and a new close obstacle is applied in one scan.
"""

from typing import List, Tuple, Dict, Optional
import numpy as np


SETTORI_8 = [
    "Fronte", "Front-DX", "Destra", "Retro-DX",
    "Retro", "Retro-SX", "Sinistra", "Front-SX"
]

SECTOR_MAP_EN = {
    "Fronte": "FRONT",
    "Front-DX": "FRONT_RIGHT",
    "Destra": "RIGHT",
    "Retro-DX": "REAR_RIGHT",
    "Retro": "REAR",
    "Retro-SX": "REAR_LEFT",
    "Sinistra": "LEFT",
    "Front-SX": "FRONT_LEFT"
}


def pulisci_outlier(dati: List[float]) -> List[float]:
    """Filters statistical outliers using 1.2 * IQR rule."""
    if len(dati) < 4:
        return dati
    q25, q75 = np.percentile(dati, [25, 75])
    iqr = q75 - q25
    lim_inf = q25 - (1.2 * iqr)
    lim_sup = q75 + (1.2 * iqr)
    puliti = [x for x in dati if lim_inf <= x <= lim_sup]
    return puliti if len(puliti) > 0 else dati


def near_distance_m(dati: List[float]) -> float:
    """Ostacolo più vicino confermato, non la mediana del settore.

    Un foglio copre solo una fetta dei 45°: la mediana (e il filtro IQR) restano
    sul muro dietro e il foglio sparisce. Si usa un percentile basso (~10°),
    scartando il singolo ritorno più corto così uno speckle isolato non scatena lo stop.
    """
    s = sorted(float(x) for x in dati)
    if len(s) <= 2:
        return s[0]
    idx = max(1, int(round(0.10 * (len(s) - 1))))
    return s[min(idx, len(s) - 1)]


MIN_BODY_CLEARANCE_M = 0.05
MAX_LIDAR_RANGE_M = 12.0
ARROW_HEADING_HALF_DEG = 15.0

class SectorAnalyzer:
    """
    Analyzes polar coordinates (angle_deg, dist_m) across 8 sectors with EMA smoothing.
    """

    def __init__(
        self,
        warning_threshold_m: float = 1.2,
        emergency_threshold_m: float = 0.35,
        ema_alpha: float = 0.45
    ):
        self.warning_threshold_m = warning_threshold_m
        self.emergency_threshold_m = emergency_threshold_m
        self.ema_alpha = ema_alpha
        self.sector_memory: Dict[str, Optional[float]] = {s: None for s in SETTORI_8}
        self.sector_timeouts: Dict[str, int] = {s: 0 for s in SETTORI_8}
        self._direct_front_memory: Optional[float] = None

    @staticmethod
    def determina_settore(angolo_deg: float) -> str:
        """Categorizes 0..360 angle into 8 45-degree sectors centered on robot axes."""
        a = angolo_deg % 360.0
        if (a >= 337.5) or (a < 22.5):
            return "Fronte"
        elif 22.5 <= a < 67.5:
            return "Front-DX"
        elif 67.5 <= a < 112.5:
            return "Destra"
        elif 112.5 <= a < 157.5:
            return "Retro-DX"
        elif 157.5 <= a < 202.5:
            return "Retro"
        elif 202.5 <= a < 247.5:
            return "Retro-SX"
        elif 247.5 <= a < 292.5:
            return "Sinistra"
        elif 292.5 <= a < 337.5:
            return "Front-SX"
        return "Fronte"

    def analyze_scan(self, points: List[Tuple[float, float]]) -> Dict:
        """
        Processes list of (angle_deg, distance_m) points.
        Returns closest obstacle, per-sector filtered min distances, and trigger decision.
        """
        punti_settori: Dict[str, List[float]] = {s: [] for s in SETTORI_8}
        direct_front_pts: List[float] = []

        for angle, dist in points:
            # Ignora eco sul corpo del robot (< 22cm: fotocamere, cavi, supporti)
            if MIN_BODY_CLEARANCE_M <= dist <= MAX_LIDAR_RANGE_M:
                settore = self.determina_settore(angle)
                punti_settori[settore].append(dist)
                # Dritto perfetto lungo la freccia fisica (±15° centrato su 0°)
                a = angle % 360.0
                if a <= ARROW_HEADING_HALF_DEG or a >= (360.0 - ARROW_HEADING_HALF_DEG):
                    direct_front_pts.append(dist)

        sector_mins: Dict[str, Optional[float]] = {}
        sector_mins_en: Dict[str, Optional[float]] = {}
        min_overall_dist = 99.0
        closest_sector = "Fronte"

        for s in SETTORI_8:
            pts = punti_settori[s]
            if len(pts) > 0:
                dist_near = near_distance_m(pts)
                self.sector_timeouts[s] = 0

                prev = self.sector_memory[s]
                if prev is None:
                    self.sector_memory[s] = dist_near
                else:
                    # Un ostacolo che entra (foglio, mano) deve comparire subito.
                    # Se la distanza aumenta, si segue comunque in fretta.
                    if dist_near + 0.12 < prev:
                        a = 0.90
                    elif dist_near > prev:
                        a = 0.50 if (dist_near - prev) < 1.0 else 0.75
                    else:
                        a = self.ema_alpha
                    self.sector_memory[s] = (a * dist_near) + ((1.0 - a) * prev)

                dist_val = round(self.sector_memory[s], 2)
                sector_mins[s] = dist_val
                sector_mins_en[SECTOR_MAP_EN[s]] = dist_val

                if dist_val < min_overall_dist:
                    min_overall_dist = dist_val
                    closest_sector = s
            else:
                self.sector_timeouts[s] += 1
                # Timeout più reattivo (3 sweep = ~300ms senza ostacolo)
                if self.sector_timeouts[s] > 3:
                    self.sector_memory[s] = None
                    sector_mins[s] = None
                    sector_mins_en[SECTOR_MAP_EN[s]] = None
                elif self.sector_memory[s] is not None:
                    dist_val = round(self.sector_memory[s], 2)
                    sector_mins[s] = dist_val
                    sector_mins_en[SECTOR_MAP_EN[s]] = dist_val
                else:
                    sector_mins[s] = None
                    sector_mins_en[SECTOR_MAP_EN[s]] = None

        # Dritto esatto lungo la freccia fisica con smoothing EMA
        if direct_front_pts:
            direct_near = near_distance_m(direct_front_pts)
            if self._direct_front_memory is None:
                self._direct_front_memory = direct_near
            else:
                if direct_near + 0.12 < self._direct_front_memory:
                    a_df = 0.90
                elif direct_near <= self._direct_front_memory:
                    a_df = self.ema_alpha
                else:
                    a_df = 0.55
                self._direct_front_memory = (a_df * direct_near) + ((1.0 - a_df) * self._direct_front_memory)
            front_dist = round(float(self._direct_front_memory), 2)
        else:
            front_dist = sector_mins.get("Fronte") or 99.0

        has_alert = min_overall_dist < self.warning_threshold_m
        is_emergency = min_overall_dist < self.emergency_threshold_m

        sector_angles = {
            "Fronte": 0.0,
            "Front-DX": 45.0,
            "Destra": 90.0,
            "Retro-DX": 135.0,
            "Retro": 180.0,
            "Retro-SX": -135.0,
            "Sinistra": -90.0,
            "Front-SX": -45.0,
        }
        target_angle = sector_angles.get(closest_sector, 0.0)

        return {
            "has_alert": has_alert,
            "is_emergency": is_emergency,
            "min_distance_m": min_overall_dist if min_overall_dist < 90 else None,
            "front_distance_m": front_dist if front_dist < 90 else None,
            "target_angle_deg": target_angle,
            "target_sector": closest_sector,
            "target_sector_en": SECTOR_MAP_EN.get(closest_sector, "FRONT"),
            "sector_mins": sector_mins,
            "sector_mins_en": sector_mins_en
        }
