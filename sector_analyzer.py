#!/usr/bin/env python3
"""
M.A.R. — LiDAR Sector Analyzer & Proximity Trigger
Segments 360-degree LiDAR point clouds into 8 operational sectors (Fronte, Front-DX, etc.),
applies statistical IQR outlier filtering, Exponential Moving Average (EMA) smoothing,
and triggers active perception saccades and emergency obstacle stops.
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


class SectorAnalyzer:
    """
    Analyzes polar coordinates (angle_deg, dist_m) across 8 sectors with EMA smoothing.
    """

    def __init__(
        self,
        warning_threshold_m: float = 1.2,
        emergency_threshold_m: float = 0.30,
        ema_alpha: float = 0.35
    ):
        self.warning_threshold_m = warning_threshold_m
        self.emergency_threshold_m = emergency_threshold_m
        self.ema_alpha = ema_alpha
        self.sector_memory: Dict[str, Optional[float]] = {s: None for s in SETTORI_8}
        self.sector_timeouts: Dict[str, int] = {s: 0 for s in SETTORI_8}

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

        for angle, dist in points:
            if 0.05 <= dist <= 12.0:
                settore = self.determina_settore(angle)
                punti_settori[settore].append(dist)

        sector_mins: Dict[str, Optional[float]] = {}
        sector_mins_en: Dict[str, Optional[float]] = {}
        min_overall_dist = 99.0
        closest_sector = "Fronte"
        target_angle = 0.0

        for s in SETTORI_8:
            pts = punti_settori[s]
            if len(pts) > 0:
                puliti = pulisci_outlier(pts)
                dist_med = float(np.median(puliti))
                self.sector_timeouts[s] = 0

                # EMA filter
                if self.sector_memory[s] is None:
                    self.sector_memory[s] = dist_med
                else:
                    self.sector_memory[s] = (self.ema_alpha * dist_med) + ((1.0 - self.ema_alpha) * self.sector_memory[s])

                dist_val = round(self.sector_memory[s], 2)
                sector_mins[s] = dist_val
                sector_mins_en[SECTOR_MAP_EN[s]] = dist_val

                if dist_val < min_overall_dist:
                    min_overall_dist = dist_val
                    closest_sector = s
            else:
                self.sector_timeouts[s] += 1
                if self.sector_timeouts[s] > 6:
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

        has_alert = min_overall_dist < self.warning_threshold_m
        is_emergency = min_overall_dist < self.emergency_threshold_m

        # Estimate target theta towards closest obstacle
        if closest_sector == "Fronte":
            target_angle = 0.0
        elif closest_sector == "Front-DX":
            target_angle = 45.0
        elif closest_sector == "Destra":
            target_angle = 90.0
        elif closest_sector == "Front-SX":
            target_angle = -45.0
        elif closest_sector == "Sinistra":
            target_angle = -90.0

        front_dist = sector_mins.get("Fronte") or 99.0

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
