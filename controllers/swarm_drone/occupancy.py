"""Log-odds occupancy grid in the SLAM frame. numpy only."""
import math

import numpy as np

L_OCC, L_FREE, L_MIN, L_MAX = 0.9, -0.35, -4.0, 4.0


class OccupancyGrid:
    def __init__(self, size_m=300.0, res=0.25):
        self.res = res
        self.n = int(round(size_m / res))
        self.o = -self.n * res / 2.0           # world coordinate of the grid's lower-left corner
        self.L = np.zeros((self.n, self.n), dtype=np.float32)    # [ix, iy], 0 = unknown

    # ---- coordinates ----
    def to_cell(self, xy):
        return np.floor((np.asarray(xy) - self.o) / self.res).astype(np.int64)

    def to_world(self, ij):
        return (np.asarray(ij, dtype=np.float64) + 0.5) * self.res + self.o

    def inside(self, ij):
        ij = np.asarray(ij)
        return (ij[..., 0] >= 0) & (ij[..., 1] >= 0) & (ij[..., 0] < self.n) & (ij[..., 1] < self.n)

    # ---- update ----
    def integrate(self, x, y, yaw, hits_body, miss_body=None):
        """Ray-trace one scan taken at pose (x, y, yaw); points are in the level body frame."""
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s], [s, c]])
        origin = np.array([x, y])
        free_cells, occ_cells = [], []
        for pts, is_hit in ((hits_body, True), (miss_body, False)):
            if pts is None or len(pts) == 0:
                continue
            w = pts @ rot.T + origin
            vec = w - origin
            length = np.hypot(vec[:, 0], vec[:, 1])
            step = self.res * 0.5
            m = int(length.max() / step) + 1
            d = np.arange(m) * step
            valid = d[None, :] < (length[:, None] - (self.res if is_hit else 0.0))
            frac = d[None, :] / np.maximum(length[:, None], 1e-9)
            samp = origin + vec[:, None, :] * frac[:, :, None]
            cells = self.to_cell(samp[valid])
            cells = cells[self.inside(cells)]
            free_cells.append(cells[:, 0] * self.n + cells[:, 1])
            if is_hit:
                oc = self.to_cell(w)
                oc = oc[self.inside(oc)]
                occ_cells.append(oc[:, 0] * self.n + oc[:, 1])
        flat = self.L.reshape(-1)
        if free_cells:
            f = np.unique(np.concatenate(free_cells))
            flat[f] = np.maximum(flat[f] + L_FREE, L_MIN)
        if occ_cells:
            o = np.unique(np.concatenate(occ_cells))
            flat[o] = np.minimum(flat[o] + L_OCC, L_MAX)

    # ---- queries ----
    def occupied(self, thresh=0.8):
        return self.L > thresh

    def likelihood_patch(self, cx, cy, half):
        """Blurred occupancy around (cx, cy) for scan matching -> (patch, (i0, j0))."""
        c = self.to_cell((cx, cy))
        h = int(half / self.res)
        i0, j0 = max(int(c[0]) - h, 0), max(int(c[1]) - h, 0)
        i1, j1 = min(int(c[0]) + h, self.n), min(int(c[1]) + h, self.n)
        a = np.clip(self.L[i0:i1, j0:j1], 0.0, L_MAX)
        for _ in range(2):                               # two 3x3 box blurs ~ gaussian
            p = np.pad(a, 1, mode="edge")
            a = sum(p[1 + di:1 + di + a.shape[0], 1 + dj:1 + dj + a.shape[1]]
                    for di in (-1, 0, 1) for dj in (-1, 0, 1)) / 9.0
        return a, (i0, j0)
