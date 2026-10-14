#!/usr/bin/env python3
"""
M.A.R. — LiDAR Sector Analyzer & Proximity Trigger
Segments 360-degree LiDAR point cloud into operational sectors and computes
the azimuth angle theta of the closest obstacle for active perception saccade.
"""

from typing import List, Tuple, Dict, Optional
import math


class SectorAnalyzer:
    """
    Analyzes polar coordinates (angle_deg, dist_m) to detect nearby obstacles.
    """

    def __init__(self, warning_threshold_m: float = 1.2, emergency_threshold_m: float = 0.3):
        self.warning_threshold_m = warning_threshold_m
        self.emergency_threshold_m = emergency_threshold_m

    @staticmethod
    def get_sector_name(angle_deg: float) -> str:
        """Categorizes angle (-180 to +180 or 0 to 360) into standard sectors."""
        # Normalize to [-180, 180] where 0 is robot forward heading
        a = (angle_deg + 180) % 360 - 180

        if -20 <= a <= 20:
            return "FRONT"
        elif 20 < a <= 60:
            return "FRONT_RIGHT"
        elif 60 < a <= 120:
            return "RIGHT"
        elif -60 <= a < -20:
            return "FRONT_LEFT"
        elif -120 <= a < -60:
            return "LEFT"
        else:
            return "REAR"

    def analyze_scan(self, points: List[Tuple[float, float]]) -> Dict:
        """
        Processes list of (angle_deg, distance_m) points.
        Returns closest obstacle, per-sector min distances, and trigger decision.
        """
        sector_mins: Dict[str, float] = {
            "FRONT": 99.0,
            "FRONT_LEFT": 99.0,
            "FRONT_RIGHT": 99.0,
            "LEFT": 99.0,
            "RIGHT": 99.0,
            "REAR": 99.0,
        }

        closest_point: Optional[Tuple[float, float, str]] = None
        min_overall_dist = 99.0

        for angle, dist in points:
            if dist <= 0.05 or dist > 15.0:
                continue  # Ignore sensor noise and extreme ranges

            sector = self.get_sector_name(angle)
            if dist < sector_mins[sector]:
                sector_mins[sector] = dist

            if dist < min_overall_dist:
                min_overall_dist = dist
                closest_point = (angle, dist, sector)

        has_alert = min_overall_dist < self.warning_threshold_m
        is_emergency = min_overall_dist < self.emergency_threshold_m

        result = {
            "has_alert": has_alert,
            "is_emergency": is_emergency,
            "min_distance_m": round(min_overall_dist, 3) if min_overall_dist < 90 else None,
            "sector_mins": {k: round(v, 2) if v < 90 else None for k, v in sector_mins.items()}
        }

        if closest_point:
            result["target_angle_deg"] = round(closest_point[0], 1)
            result["target_sector"] = closest_point[2]

        return result
