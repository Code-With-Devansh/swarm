"""Scan-to-map 2D SLAM front end for the leader (no loop closure yet).

Each scan is matched against the current occupancy grid by hill-climbing the pose
(x, y, yaw) around a motion prior, then integrated into the grid. This is the
CoreSLAM / BreezySLAM idea written in numpy so it needs no compiled extension.

Yaw comes from the IMU (absolute, drift-free in Webots), so the matcher only
refines it by a few hundredths of a radian. x, y are what actually drift.
"""
import math

import numpy as np

STEPS = [(0.4, 0.03), (0.2, 0.015), (0.1, 0.008), (0.05, 0.004)]   # (metres, radians), coarse -> fine


class Slam2D:
    def __init__(self, grid, max_range=40.0, prior_weight=0.02):
        self.grid = grid
        self.max_range = max_range
        self.lam = prior_weight
        self.pose = None                  # (x, y, yaw) in the SLAM frame
        self.vel = (0.0, 0.0)
        self.t_prev = None
        self.quality = 0.0
        self.scans = 0

    def reset(self, x, y, yaw, t=None):
        """Set the pose without matching (start-up, or a new leader taking over)."""
        self.pose = (x, y, yaw)
        self.vel = (0.0, 0.0)
        self.t_prev = t

    def _score(self, patch, org, pts, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        w = pts @ np.array([[c, s], [-s, c]]) + (x, y)
        ij = np.floor((w - self.grid.o) / self.grid.res).astype(np.int64) - org
        ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < patch.shape[0]) & (ij[:, 1] < patch.shape[1])
        return float(patch[ij[ok, 0], ij[ok, 1]].sum())

    def update(self, t, hits, miss=None, yaw_imu=None, vel_hint=None):
        """Feed one scan. Returns (x, y, yaw, quality 0..1)."""
        if self.pose is None:
            raise RuntimeError("call reset() with the start pose first")
        x0, y0, yaw0 = self.pose
        dt = 0.0 if self.t_prev is None else max(0.0, t - self.t_prev)
        v = vel_hint if vel_hint is not None else self.vel
        px, py = x0 + v[0] * dt, y0 + v[1] * dt
        pyaw = yaw_imu if yaw_imu is not None else yaw0
        self.t_prev = t

        if self.scans > 0 and len(hits) >= 20:
            patch, org = self.grid.likelihood_patch(px, py, self.max_range + 3.0)
            org = np.array(org)
            best = (px, py, pyaw)
            bs = self._score(patch, org, hits, *best) - self.lam * 0.0
            for sxy, syaw in STEPS:
                for _ in range(25):
                    improved = False
                    cand = [(sxy, 0, 0), (-sxy, 0, 0), (0, sxy, 0), (0, -sxy, 0)]
                    if yaw_imu is not None:
                        cand += [(0, 0, syaw), (0, 0, -syaw)]
                    for dx, dy, dth in cand:
                        c = (best[0] + dx, best[1] + dy, best[2] + dth)
                        if abs(c[2] - pyaw) > 0.06:        # trust the IMU yaw
                            continue
                        pen = self.lam * ((c[0] - px) ** 2 + (c[1] - py) ** 2)
                        sc = self._score(patch, org, hits, *c) - pen * len(hits)
                        if sc > bs + 1e-9:
                            best, bs, improved = c, sc, True
                    if not improved:
                        break
            self.quality = min(1.0, self._score(patch, org, hits, *best) / (len(hits) * 2.0))
            # reject matches against an almost empty map: keep the prior instead
            nx, ny, nyaw = best if self.quality > 0.15 else (px, py, pyaw)
        else:
            nx, ny, nyaw = px, py, pyaw
            self.quality = 0.0

        if dt > 1e-6 and self.scans > 0:
            a = 0.5                                        # low-pass the velocity estimate
            self.vel = (a * self.vel[0] + (1 - a) * (nx - x0) / dt,
                        a * self.vel[1] + (1 - a) * (ny - y0) / dt)
        self.pose = (nx, ny, nyaw)
        self.grid.integrate(nx, ny, nyaw, hits, miss)
        self.scans += 1
        return nx, ny, nyaw, self.quality
