"""Live map for demos: draws the leader's occupancy grid, A* route and trail on a Webots Display.

numpy only. Red = what the lidar saw as obstacles, light = mapped free space, dark = unknown,
green = the A* route, cyan = where the leader has been (SLAM pose), yellow = patrol waypoints.
In Webots: right-click the 3D view -> Overlays -> droneN -> map (the leader draws; after a failover
the new leader draws on its own display).
"""
import math

import numpy as np

UNKNOWN, FREE, OCC = (45, 45, 50), (200, 205, 210), (230, 50, 50)
PATH, TRAIL, WAYPT, ME = (40, 220, 90), (60, 200, 255), (255, 210, 40), (255, 255, 255)


class MapView:
    def __init__(self, display, rgba_format, px=480, center=(-20.0, 0.0)):
        self.d, self.fmt, self.px = display, rgba_format, px
        self.cx, self.cy = center
        self.trail = []

    def _pix(self, x, y, res):
        half = self.px * res / 2
        return (x - (self.cx - half)) / res, (self.cy + half - y) / res      # col, row

    def _dot(self, img, x, y, r, col, res):
        c, rw = self._pix(x, y, res)
        c0, r0 = int(round(c)), int(round(rw))
        img[max(r0 - r, 0):max(r0 + r + 1, 0), max(c0 - r, 0):max(c0 + r + 1, 0), :3] = col

    def _line(self, img, pts, col, res, r=1):
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            n = max(2, int(math.hypot(x1 - x0, y1 - y0) / (res * 0.5)))
            for k in range(n + 1):
                f = k / n
                self._dot(img, x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, r, col, res)

    def draw(self, grid, pose, path, waypoints, goal):
        res, px = grid.res, self.px
        half = px * res / 2
        i0 = int(round((self.cx - half - grid.o) / res))
        j0 = int(round((self.cy - half - grid.o) / res))
        patch = np.full((px, px), 0.0, dtype=np.float32)
        a, b = max(i0, 0), max(j0, 0)
        sub = grid.L[a:i0 + px, b:j0 + px]
        patch[a - i0:a - i0 + sub.shape[0], b - j0:b - j0 + sub.shape[1]] = sub
        L = patch.T[::-1]                                  # rows = y (north up), cols = x
        img = np.full((px, px, 4), 255, dtype=np.uint8)
        img[..., :3] = UNKNOWN
        img[L < -0.3, :3] = FREE
        img[L > 0.8, :3] = OCC
        for wp in waypoints:
            self._dot(img, wp[0], wp[1], 3, WAYPT, res)
        self.trail.append((pose[0], pose[1]))
        self.trail = self.trail[-3000:]
        self._line(img, self.trail[::5], TRAIL, res)
        if path:
            self._line(img, [(pose[0], pose[1])] + list(path), PATH, res, r=1)
        self._dot(img, pose[0], pose[1], 4, ME, res)
        ir = self.d.imageNew(img.tobytes(), self.fmt, px, px)
        self.d.imagePaste(ir, 0, 0, False)
        self.d.imageDelete(ir)
        self.d.setColor(0xFFFFFF)
        self.d.drawText("red: lidar obstacles   green: A* route   cyan: trail   yellow: waypoints", 4, 4)
