#!/usr/bin/env python3
"""
M.A.R. — Real-Time 2D LiDAR SLAM & Spatial Localization Engine
Performs simultaneous localization and mapping (SLAM) using 360-degree point clouds:
  - Robust 2D scan-to-map ICP (Iterative Closest Point) alignment
  - Real-time 3-DoF pose estimation (X_m, Y_m, Theta_deg)
  - Persistent global spatial hash point cloud (5cm resolution)
  - Odometry and path trajectory history
  - Room bounding box & metric exploration metrics
"""

from __future__ import annotations
import time
import math
from typing import List, Tuple, Dict, Optional
import numpy as np


class LidarSlamEngine:
    """
    Motore SLAM 2D per robot quadrupede / rover basato su LiDAR 360° LD06.
    Traccia in modo continuo la posizione del robot nella stanza (X, Y, Theta)
    e costruisce la mappa persistente dei muri e degli ostacoli dell'ambiente.
    """

    def __init__(
        self,
        grid_resolution_m: float = 0.05,  # 5 cm per cella
        max_map_points: int = 15000,
        icp_max_iterations: int = 6,
        icp_tolerance_m: float = 0.005,
    ):
        self.res = grid_resolution_m
        self.max_map_points = max_map_points
        self.icp_max_iterations = icp_max_iterations
        self.icp_tolerance_m = icp_tolerance_m

        # Stato posa corrente nel frame mondo (metri, radianti)
        self.pose_x: float = 0.0
        self.pose_y: float = 0.0
        self.pose_theta: float = 0.0

        # Velocità stimate
        self.vx: float = 0.0
        self.vy: float = 0.0
        self.vtheta: float = 0.0
        self.total_distance_m: float = 0.0
        self.last_pose_time: float = 0.0

        # Mappa globale spaziale: chiave (gx, gy) -> [x_sum, y_sum, hit_count, last_seen]
        self.spatial_map: Dict[Tuple[int, int], List[float]] = {}

        # Traiettoria percorsa [(x, y, theta, timestamp), ...]
        self.trajectory: List[Tuple[float, float, float, float]] = []
        self._last_traj_add_pos: Tuple[float, float] = (0.0, 0.0)

        # Scansione di riferimento precedente (array Nx2)
        self.prev_cloud_world: Optional[np.ndarray] = None
        self.is_initialized: bool = False

    def _polar_to_cartesian(self, scan_points: List[Tuple[float, float]]) -> np.ndarray:
        """Converte [(angolo_deg, dist_m), ...] nel frame locale robot (Nx2)."""
        pts = []
        for ang_deg, dist_m in scan_points:
            if not (0.15 <= dist_m <= 10.0):
                continue
            rad = math.radians(ang_deg)
            # Frame robot: 0° = Fronte (+X), 90° = Sinistra (+Y)
            xr = dist_m * math.cos(rad)
            yr = dist_m * math.sin(rad)
            pts.append([xr, yr])
        return np.array(pts, dtype=np.float32) if pts else np.empty((0, 2), dtype=np.float32)

    def _transform_points(self, points_local: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
        """Trasforma punti dal frame robot al frame mondo usando la posa (x, y, theta)."""
        if len(points_local) == 0:
            return points_local
        c, s = math.cos(theta), math.sin(theta)
        R = np.array([[c, -s], [s, c]], dtype=np.float32)
        return (points_local @ R.T) + np.array([x, y], dtype=np.float32)

    def _fast_icp_step(self, current_local: np.ndarray, init_pose: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Allinea la scansione corrente alla mappa globale con ICP 2D rapido.
        Restituisce la posa ottimizzata (x, y, theta).
        """
        if len(self.spatial_map) < 30 or len(current_local) < 15:
            return init_pose

        # Estrai punti mappa di riferimento vicini
        map_keys = list(self.spatial_map.keys())
        map_pts = np.array([[k[0] * self.res, k[1] * self.res] for k in map_keys], dtype=np.float32)

        cx, cy, cth = init_pose

        for _ in range(self.icp_max_iterations):
            cur_world = self._transform_points(current_local, cx, cy, cth)

            # Nearest neighbor rapido tramite griglia
            matched_map = []
            matched_cur = []

            for i, p in enumerate(cur_world):
                gx, gy = int(round(p[0] / self.res)), int(round(p[1] / self.res))
                # Cerca nel raggio di 2 celle (10cm)
                best_dist = 0.35  # max 35 cm per match valido
                best_pt = None
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        k = (gx + dx, gy + dy)
                        if k in self.spatial_map:
                            entry = self.spatial_map[k]
                            cand = np.array([entry[0] / entry[2], entry[1] / entry[2]], dtype=np.float32)
                            d = np.linalg.norm(cand - p)
                            if d < best_dist:
                                best_dist = d
                                best_pt = cand
                if best_pt is not None:
                    matched_cur.append(current_local[i])
                    matched_map.append(best_pt)

            if len(matched_cur) < 12:
                break

            P = np.array(matched_cur, dtype=np.float32)
            Q = np.array(matched_map, dtype=np.float32)

            # Closed-form SVD 2D alignment tra P e Q
            # Vogliamo trovare R, t tale che R*P + t ≈ Q
            p_mean = np.mean(P, axis=0)
            q_mean = np.mean(Q, axis=0)
            P_shift = P - p_mean
            Q_shift = Q - q_mean

            H = P_shift.T @ Q_shift
            U, S, Vt = np.linalg.svd(H)
            R_est = Vt.T @ U.T
            if np.linalg.det(R_est) < 0:
                Vt[1, :] *= -1
                R_est = Vt.T @ U.T

            d_th = math.atan2(R_est[1, 0], R_est[0, 0])
            t_est = q_mean - (p_mean @ R_est.T)

            # Aggiornamento posa con smorzamento
            cx = (0.75 * t_est[0]) + (0.25 * cx)
            cy = (0.75 * t_est[1]) + (0.25 * cy)
            cth = (0.75 * d_th) + (0.25 * cth)

        return cx, cy, cth

    def _integrate_into_map(self, world_points: np.ndarray, now: float):
        """Integra i punti registrati nel frame mondo nella spatial hash grid."""
        for p in world_points:
            gx = int(round(p[0] / self.res))
            gy = int(round(p[1] / self.res))
            k = (gx, gy)
            if k in self.spatial_map:
                entry = self.spatial_map[k]
                entry[0] += p[0]
                entry[1] += p[1]
                entry[2] += 1.0
                entry[3] = now
            else:
                if len(self.spatial_map) < self.max_map_points:
                    self.spatial_map[k] = [float(p[0]), float(p[1]), 1.0, now]

    def update_scan(
        self,
        scan_points: List[Tuple[float, float]],
        imu_yaw_deg: Optional[float] = None,
        now: Optional[float] = None
    ) -> Dict[str, object]:
        """
        Processa una scansione 360° LiDAR:
        - Estima la nuova posa del robot
        - Aggiorna la mappa globale della stanza
        - Restituisce la telemetria SLAM completa
        """
        t = time.time() if now is None else float(now)
        local_pts = self._polar_to_cartesian(scan_points)

        if len(local_pts) < 15:
            return self.get_status()

        if not self.is_initialized:
            # Primo frame: origine mondo (0, 0, 0)
            self.pose_x = 0.0
            self.pose_y = 0.0
            self.pose_theta = 0.0
            self.last_pose_time = t
            world_pts = self._transform_points(local_pts, 0.0, 0.0, 0.0)
            self._integrate_into_map(world_pts, t)
            self.prev_cloud_world = world_pts
            self.trajectory.append((0.0, 0.0, 0.0, t))
            self.is_initialized = True
            return self.get_status()

        dt = max(0.01, t - self.last_pose_time)

        # Predizione iniziale della posa da odometria / velocità costante
        pred_x = self.pose_x + (self.vx * dt)
        pred_y = self.pose_y + (self.vy * dt)
        pred_th = self.pose_theta + (self.vtheta * dt)

        # Se disponibile il dato IMU (yaw assoluto o gyro), vincola la rotazione
        if imu_yaw_deg is not None:
            imu_rad = math.radians(imu_yaw_deg)
            # Blend 50/50 tra predizione e IMU
            pred_th = 0.5 * pred_th + 0.5 * imu_rad

        # Allineamento ICP con la mappa esistente
        new_x, new_y, new_th = self._fast_icp_step(local_pts, (pred_x, pred_y, pred_th))

        # Calcolo velocità lineari e angolari
        dx = new_x - self.pose_x
        dy = new_y - self.pose_y
        d_dist = math.hypot(dx, dy)
        d_th = new_th - self.pose_theta

        self.vx = dx / dt
        self.vy = dy / dt
        self.vtheta = d_th / dt
        self.total_distance_m += d_dist

        # Aggiorna stato
        self.pose_x = new_x
        self.pose_y = new_y
        self.pose_theta = new_th
        self.last_pose_time = t

        # Aggiungi a traiettoria se il robot si è mosso di almeno 5 cm o 3 gradi
        if math.hypot(new_x - self._last_traj_add_pos[0], new_y - self._last_traj_add_pos[1]) >= 0.05:
            self.trajectory.append((round(new_x, 3), round(new_y, 3), round(math.degrees(new_th), 1), round(t, 2)))
            self._last_traj_add_pos = (new_x, new_y)
            if len(self.trajectory) > 500:
                self.trajectory = self.trajectory[-500:]

        # Trasforma la scansione corrente nella posa stimata e integra nella mappa globale
        world_pts = self._transform_points(local_pts, new_x, new_y, new_th)
        self._integrate_into_map(world_pts, t)

        return self.get_status()

    def get_status(self) -> Dict[str, object]:
        """Restituisce telemetria di posa e metriche della stanza."""
        th_deg = round(math.degrees(self.pose_theta) % 360.0, 1)

        # Calcola bounding box della stanza mappata
        if self.spatial_map:
            keys = np.array(list(self.spatial_map.keys()), dtype=np.float32) * self.res
            min_x, min_y = float(np.min(keys[:, 0])), float(np.min(keys[:, 1]))
            max_x, max_y = float(np.max(keys[:, 0])), float(np.max(keys[:, 1]))
            room_w = round(max_x - min_x, 2)
            room_l = round(max_y - min_y, 2)
            area_m2 = round(room_w * room_l, 2)
        else:
            min_x = min_y = max_x = max_y = room_w = room_l = area_m2 = 0.0

        return {
            "x_m": round(self.pose_x, 3),
            "y_m": round(self.pose_y, 3),
            "theta_deg": th_deg,
            "linear_speed_mps": round(math.hypot(self.vx, self.vy), 2),
            "angular_speed_dps": round(math.degrees(self.vtheta), 1),
            "distance_traveled_m": round(self.total_distance_m, 2),
            "map_points_count": len(self.spatial_map),
            "trajectory_points": len(self.trajectory),
            "room_bounds_m": {
                "min_x": round(min_x, 2),
                "max_x": round(max_x, 2),
                "min_y": round(min_y, 2),
                "max_y": round(max_y, 2),
                "width_m": room_w,
                "length_m": room_l,
                "area_m2": area_m2,
            }
        }

    def get_downsampled_cloud(self, max_points: int = 1200) -> List[List[float]]:
        """Restituisce la nuvola di punti della stanza per rendering grafico (Nx2)."""
        if not self.spatial_map:
            return []
        items = list(self.spatial_map.values())
        # Filtra celle con almeno 2 hit (conferma muro/ostacolo fisso)
        confirmed = [item for item in items if item[2] >= 2.0]
        pts_source = confirmed if len(confirmed) >= 50 else items

        step = max(1, len(pts_source) // max_points)
        sampled = []
        for i in range(0, len(pts_source), step):
            entry = pts_source[i]
            x = round(entry[0] / entry[2], 2)
            y = round(entry[1] / entry[2], 2)
            sampled.append([x, y])
            if len(sampled) >= max_points:
                break
        return sampled

    def get_full_map_payload(self) -> Dict[str, object]:
        """Payload completo per la dashboard web / MQTT consumer."""
        status = self.get_status()
        cloud = self.get_downsampled_cloud()
        return {
            "timestamp": time.time(),
            "pose": {
                "x": status["x_m"],
                "y": status["y_m"],
                "theta": status["theta_deg"],
            },
            "speed": {
                "linear": status["linear_speed_mps"],
                "angular": status["angular_speed_dps"],
                "distance": status["distance_traveled_m"],
            },
            "room": status["room_bounds_m"],
            "trajectory": [[p[0], p[1]] for p in self.trajectory[-150:]],
            "point_count": len(cloud),
            "cloud": cloud,
        }
