#!/usr/bin/env python3
"""
M.A.R. — Real-Time 2D LiDAR SLAM & Spatial Localization Engine (vectorized)

Scan-to-map ICP 2D su griglia spaziale a 5 cm, completamente vettorizzato con NumPy
(nessun loop Python per punto): adatto a 10 Hz su Raspberry Pi 5.

Convenzioni (coerenti con SectorAnalyzer e con la dashboard):
  • angolo LiDAR 0° = Fronte, cresce in senso ORARIO (45° = Front-DX, 270° = Sinistra)
  • frame robot/mondo destrorso: X avanti, Y SINISTRA  →  x = d·cos(a),  y = −d·sin(a)
  • theta (posa) positivo = rotazione antioraria (verso sinistra)

Robustezza:
  • prior di rotazione dall'IMU (yaw) quando disponibile, altrimenti velocità costante
  • integrazione nella mappa solo a "keyframe" (≥5 cm / ≥3° / ≥1 s) e solo con match ICP
    sufficiente → niente inquinamento della mappa quando l'allineamento è dubbio
  • limite al salto di posa per scansione (rigetta divergenze ICP)
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

_OFFSETS = np.array([[dx, dy] for dx in (-1, 0, 1) for dy in (-1, 0, 1)], dtype=np.int32)


class LidarSlamEngine:
    """Motore SLAM 2D per robot quadrupede basato su LiDAR 360° (LD06)."""

    def __init__(
        self,
        grid_resolution_m: float = 0.05,
        max_map_points: int = 20000,
        icp_max_iterations: int = 8,
        icp_tolerance_m: float = 0.003,
        max_match_dist_m: float = 0.30,
    ):
        self.res = float(grid_resolution_m)
        self.max_map_points = int(max_map_points)
        self.icp_max_iterations = int(icp_max_iterations)
        self.icp_tolerance_m = float(icp_tolerance_m)
        self.max_match_dist_m = float(max_match_dist_m)

        # Posa corrente (metri, radianti)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.vx = self.vy = self.vtheta = 0.0
        self.total_distance_m = 0.0
        self.last_pose_time = 0.0

        # Mappa: celle con centroide incrementale
        self._key_to_idx: Dict[Tuple[int, int], int] = {}
        self._cells = np.zeros((0, 2), dtype=np.int32)       # (gx, gy)
        self._sum = np.zeros((0, 2), dtype=np.float64)       # somma x,y
        self._hits = np.zeros((0,), dtype=np.float64)
        self._last_seen = np.zeros((0,), dtype=np.float64)

        # Griglia densa cella → indice (per NN vettorizzato); ricostruita quando serve
        self._grid: Optional[np.ndarray] = None
        self._grid_origin = np.array([0, 0], dtype=np.int32)
        self._grid_dirty = True

        # Traiettoria [(x, y, theta_deg, t)]
        self.trajectory: List[Tuple[float, float, float, float]] = []
        self._last_traj_add_pos = (0.0, 0.0)
        self._last_key_pose = (0.0, 0.0, 0.0)
        self._last_key_time = 0.0

        self.is_initialized = False
        self.match_ratio = 0.0
        self.icp_rmse_m = 0.0
        self.icp_ok = False

        # Prior IMU: offset tra yaw IMU e theta mappa (stimato all'avvio, poi lentamente)
        self._imu_offset_rad: Optional[float] = None

    # ------------------------------------------------------------------ geometria
    def _polar_to_cartesian(self, scan_points: List[Tuple[float, float]]) -> np.ndarray:
        """[(angolo_deg, dist_m)] → Nx2 nel frame robot (X avanti, Y sinistra), 1 punto per grado."""
        if not scan_points:
            return np.empty((0, 2), dtype=np.float32)
        arr = np.asarray(scan_points, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] < 2:
            return np.empty((0, 2), dtype=np.float32)
        ang = arr[:, 0]
        dist = arr[:, 1]
        ok = (dist >= 0.15) & (dist <= 10.0) & np.isfinite(ang) & np.isfinite(dist)
        ang = ang[ok]
        dist = dist[ok]
        if ang.size == 0:
            return np.empty((0, 2), dtype=np.float32)
        # downsample a 1°: minimo per bin (ostacolo più vicino, meno multi-echo)
        bins = (np.round(ang).astype(np.int64)) % 360
        order = np.lexsort((dist, bins))
        bins_sorted = bins[order]
        first = np.ones(bins_sorted.size, dtype=bool)
        first[1:] = bins_sorted[1:] != bins_sorted[:-1]
        sel = order[first]
        a_rad = np.radians(ang[sel])
        d = dist[sel]
        pts = np.stack([d * np.cos(a_rad), -d * np.sin(a_rad)], axis=1)
        return pts.astype(np.float32)

    @staticmethod
    def _transform_points(points_local: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
        if len(points_local) == 0:
            return points_local
        c, s = math.cos(theta), math.sin(theta)
        R = np.array([[c, -s], [s, c]], dtype=np.float32)
        return (points_local @ R.T) + np.array([x, y], dtype=np.float32)

    # ------------------------------------------------------------------ mappa
    def _centroids(self) -> np.ndarray:
        if self._hits.size == 0:
            return np.zeros((0, 2), dtype=np.float32)
        return (self._sum / self._hits[:, None]).astype(np.float32)

    def _rebuild_grid(self) -> None:
        if self._cells.shape[0] == 0:
            self._grid = None
            self._grid_dirty = False
            return
        mn = self._cells.min(axis=0) - 2
        mx = self._cells.max(axis=0) + 2
        size = (mx - mn) + 1
        # griglia densa (int32 indice, -1 = vuota); 20 m × 20 m a 5 cm = 160k celle → ok
        grid = np.full((int(size[0]), int(size[1])), -1, dtype=np.int32)
        idx = self._cells - mn
        grid[idx[:, 0], idx[:, 1]] = np.arange(self._cells.shape[0], dtype=np.int32)
        self._grid = grid
        self._grid_origin = mn.astype(np.int32)
        self._grid_dirty = False

    def _integrate_into_map(self, world_points: np.ndarray, now: float) -> None:
        if len(world_points) == 0:
            return
        cells = np.round(world_points / self.res).astype(np.int32)
        # aggrega i punti della scansione per cella (somma, conteggio)
        uniq, inv, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
        inv = inv.reshape(-1)
        sums = np.zeros((uniq.shape[0], 2), dtype=np.float64)
        np.add.at(sums, inv, world_points.astype(np.float64))

        new_cells = []
        new_sums = []
        new_counts = []
        for i in range(uniq.shape[0]):
            k = (int(uniq[i, 0]), int(uniq[i, 1]))
            j = self._key_to_idx.get(k)
            if j is None:
                if self._cells.shape[0] + len(new_cells) >= self.max_map_points:
                    continue
                self._key_to_idx[k] = self._cells.shape[0] + len(new_cells)
                new_cells.append(uniq[i])
                new_sums.append(sums[i])
                new_counts.append(float(counts[i]))
            else:
                self._sum[j] += sums[i]
                self._hits[j] += float(counts[i])
                self._last_seen[j] = now
        if new_cells:
            self._cells = np.vstack([self._cells, np.asarray(new_cells, dtype=np.int32)])
            self._sum = np.vstack([self._sum, np.asarray(new_sums, dtype=np.float64)])
            self._hits = np.concatenate([self._hits, np.asarray(new_counts, dtype=np.float64)])
            self._last_seen = np.concatenate([self._last_seen, np.full(len(new_cells), now, dtype=np.float64)])
            self._grid_dirty = True

    def _nearest_map_points(self, world_pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Per ogni punto: centroide mappa più vicino nelle 9 celle intorno. Ritorna (match_mask, Q)."""
        if self._grid_dirty:
            self._rebuild_grid()
        if self._grid is None:
            return np.zeros(len(world_pts), dtype=bool), np.zeros((len(world_pts), 2), dtype=np.float32)
        cent = self._centroids()
        cells = np.round(world_pts / self.res).astype(np.int32) - self._grid_origin  # N×2
        cand = cells[:, None, :] + _OFFSETS[None, :, :]                                # N×9×2
        H, W = self._grid.shape
        inside = (cand[..., 0] >= 0) & (cand[..., 0] < H) & (cand[..., 1] >= 0) & (cand[..., 1] < W)
        cx = np.clip(cand[..., 0], 0, H - 1)
        cy = np.clip(cand[..., 1], 0, W - 1)
        idx = self._grid[cx, cy]                                                        # N×9
        idx = np.where(inside, idx, -1)
        valid = idx >= 0
        safe_idx = np.where(valid, idx, 0)
        q = cent[safe_idx]                                                              # N×9×2
        d = np.linalg.norm(q - world_pts[:, None, :], axis=2)
        d = np.where(valid, d, np.inf)
        best = np.argmin(d, axis=1)
        best_d = d[np.arange(len(world_pts)), best]
        match = best_d < self.max_match_dist_m
        Q = q[np.arange(len(world_pts)), best]
        return match, Q

    # ------------------------------------------------------------------ ICP
    def _icp(self, local: np.ndarray, init_pose: Tuple[float, float, float]) -> Tuple[float, float, float, float, float]:
        """Ritorna (x, y, theta, match_ratio, rmse)."""
        cx, cy, cth = init_pose
        n = len(local)
        if self._cells.shape[0] < 30 or n < 15:
            return cx, cy, cth, 0.0, 0.0
        ratio = 0.0
        rmse = 0.0
        for _ in range(self.icp_max_iterations):
            world = self._transform_points(local, cx, cy, cth)
            match, Q = self._nearest_map_points(world)
            m = int(match.sum())
            ratio = m / float(n)
            if m < 12:
                break
            P = local[match].astype(np.float64)
            Qm = Q[match].astype(np.float64)
            p_mean = P.mean(axis=0)
            q_mean = Qm.mean(axis=0)
            Hm = (P - p_mean).T @ (Qm - q_mean)
            U, _, Vt = np.linalg.svd(Hm)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[1, :] *= -1
                R = Vt.T @ U.T
            th_new = math.atan2(R[1, 0], R[0, 0])
            t_new = q_mean - R @ p_mean
            dx, dy = float(t_new[0]) - cx, float(t_new[1]) - cy
            dth = (th_new - cth + math.pi) % (2 * math.pi) - math.pi
            cx, cy, cth = float(t_new[0]), float(t_new[1]), float(cth + dth)
            resid = (self._transform_points(local[match], cx, cy, cth).astype(np.float64) - Qm)
            rmse = float(np.sqrt((resid ** 2).sum(axis=1).mean()))
            if math.hypot(dx, dy) < self.icp_tolerance_m and abs(dth) < math.radians(0.1):
                break
        return cx, cy, cth, ratio, rmse

    # ------------------------------------------------------------------ API
    def update_scan(
        self,
        scan_points: List[Tuple[float, float]],
        imu_yaw_deg: Optional[float] = None,
        now: Optional[float] = None,
    ) -> Dict[str, object]:
        t = time.time() if now is None else float(now)
        local = self._polar_to_cartesian(scan_points)
        if len(local) < 15:
            return self.get_status()

        if not self.is_initialized:
            self.pose_x = self.pose_y = self.pose_theta = 0.0
            self.last_pose_time = t
            self._integrate_into_map(self._transform_points(local, 0.0, 0.0, 0.0), t)
            self.trajectory.append((0.0, 0.0, 0.0, t))
            self._last_key_pose = (0.0, 0.0, 0.0)
            self._last_key_time = t
            if imu_yaw_deg is not None:
                self._imu_offset_rad = -math.radians(imu_yaw_deg)
            self.is_initialized = True
            self.icp_ok = True
            return self.get_status()

        dt = max(0.01, min(0.5, t - self.last_pose_time))

        # Predizione: velocità costante; rotazione dall'IMU se disponibile
        pred_x = self.pose_x + self.vx * dt
        pred_y = self.pose_y + self.vy * dt
        pred_th = self.pose_theta + self.vtheta * dt
        if imu_yaw_deg is not None:
            yaw = math.radians(float(imu_yaw_deg))
            if self._imu_offset_rad is None:
                self._imu_offset_rad = self.pose_theta - yaw
            pred_th = self._imu_offset_rad + yaw

        nx, ny, nth, ratio, rmse = self._icp(local, (pred_x, pred_y, pred_th))
        self.match_ratio = ratio
        self.icp_rmse_m = rmse

        # Validazione: match sufficiente e salto plausibile per una scansione (10 Hz)
        jump = math.hypot(nx - self.pose_x, ny - self.pose_y)
        dth = (nth - self.pose_theta + math.pi) % (2 * math.pi) - math.pi
        self.icp_ok = (ratio >= 0.35) and (jump <= 0.6) and (abs(dth) <= math.radians(60.0))
        if not self.icp_ok:
            # tieni la predizione (non fidarti dell'allineamento), smorza la velocità
            nx, ny, nth = pred_x, pred_y, pred_th
            self.vx *= 0.5
            self.vy *= 0.5
            self.vtheta *= 0.5
        else:
            self.vx = (nx - self.pose_x) / dt
            self.vy = (ny - self.pose_y) / dt
            self.vtheta = dth / dt
            self.total_distance_m += jump
            if imu_yaw_deg is not None and self._imu_offset_rad is not None:
                # affina lentamente l'offset IMU→mappa
                yaw = math.radians(float(imu_yaw_deg))
                err = (nth - (self._imu_offset_rad + yaw) + math.pi) % (2 * math.pi) - math.pi
                self._imu_offset_rad += 0.05 * err

        self.pose_x, self.pose_y, self.pose_theta = float(nx), float(ny), float(nth)
        self.last_pose_time = t

        if math.hypot(nx - self._last_traj_add_pos[0], ny - self._last_traj_add_pos[1]) >= 0.05:
            self.trajectory.append((round(nx, 3), round(ny, 3), round(math.degrees(nth), 1), round(t, 2)))
            self._last_traj_add_pos = (nx, ny)
            if len(self.trajectory) > 500:
                self.trajectory = self.trajectory[-500:]

        # Keyframe: integra nella mappa solo se allineati bene e ci si è mossi (o ogni 1 s)
        kx, ky, kth = self._last_key_pose
        moved = math.hypot(nx - kx, ny - ky) >= 0.05 or abs((nth - kth + math.pi) % (2 * math.pi) - math.pi) >= math.radians(3.0)
        if self.icp_ok and (moved or (t - self._last_key_time) >= 1.0):
            self._integrate_into_map(self._transform_points(local, nx, ny, nth), t)
            self._last_key_pose = (nx, ny, nth)
            self._last_key_time = t

        return self.get_status()

    def get_status(self) -> Dict[str, object]:
        th_deg = round(float(math.degrees(self.pose_theta) % 360.0), 1) % 360.0
        if self._cells.shape[0] > 0:
            mn = self._cells.min(axis=0) * self.res
            mx = self._cells.max(axis=0) * self.res
            min_x, min_y = float(mn[0]), float(mn[1])
            max_x, max_y = float(mx[0]), float(mx[1])
            room_w = round(max_x - min_x, 2)
            room_l = round(max_y - min_y, 2)
            area_m2 = round(room_w * room_l, 2)
        else:
            min_x = min_y = max_x = max_y = room_w = room_l = area_m2 = 0.0
        return {
            "x_m": round(float(self.pose_x), 3),
            "y_m": round(float(self.pose_y), 3),
            "theta_deg": th_deg,
            "linear_speed_mps": round(float(math.hypot(self.vx, self.vy)), 2),
            "angular_speed_dps": round(float(math.degrees(self.vtheta)), 1),
            "distance_traveled_m": round(float(self.total_distance_m), 2),
            "map_points_count": int(self._cells.shape[0]),
            "trajectory_points": int(len(self.trajectory)),
            "match_ratio": round(float(self.match_ratio), 2),
            "icp_rmse_m": round(float(self.icp_rmse_m), 3),
            "icp_ok": bool(self.icp_ok),
            "frame": "x_forward_y_left",
            "room_bounds_m": {
                "min_x": round(min_x, 2), "max_x": round(max_x, 2),
                "min_y": round(min_y, 2), "max_y": round(max_y, 2),
                "width_m": float(room_w), "length_m": float(room_l), "area_m2": float(area_m2),
            },
        }

    def get_downsampled_cloud(self, max_points: int = 1200) -> List[List[float]]:
        if self._cells.shape[0] == 0:
            return []
        cent = self._centroids()
        confirmed = self._hits >= 2.0
        pts = cent[confirmed] if int(confirmed.sum()) >= 50 else cent
        step = max(1, len(pts) // max_points)
        pts = pts[::step][:max_points]
        return np.round(pts, 2).tolist()

    def get_full_map_payload(self) -> Dict[str, object]:
        status = self.get_status()
        cloud = self.get_downsampled_cloud()
        return {
            "timestamp": float(time.time()),
            "pose": {"x": float(status["x_m"]), "y": float(status["y_m"]), "theta": float(status["theta_deg"])},
            "speed": {
                "linear": float(status["linear_speed_mps"]),
                "angular": float(status["angular_speed_dps"]),
                "distance": float(status["distance_traveled_m"]),
            },
            "quality": {"match_ratio": status["match_ratio"], "icp_ok": status["icp_ok"], "rmse_m": status["icp_rmse_m"]},
            "frame": "x_forward_y_left",
            "room": status["room_bounds_m"],
            "trajectory": [[float(p[0]), float(p[1])] for p in self.trajectory[-150:]],
            "point_count": int(len(cloud)),
            "cloud": cloud,
        }
