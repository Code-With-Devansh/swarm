"""SLAM / mapping / planning tests on a synthetic 2D world (no Webots needed)."""
import math
import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "controllers" / "swarm_drone"))

from lidar_utils import beam_angles, level_points, mask_peers
from leader_nav import LeaderNav
from swarm_core import obstacle_repulsion
from occupancy import OccupancyGrid
from slam import Slam2D
from planner import plan, carrot

# circles (x, y, r) standing in for trees / windmill towers, plus four boundary walls
OBST = [(10, 4, 1.5), (14, -5, 1.2), (22, 3, 2.0), (28, -2, 1.5), (35, 6, 1.5), (18, 12, 1.0), (30, 14, 1.8)]
WALLS = [((-10, -20), (50, -20)), ((50, -20), (50, 25)), ((50, 25), (-10, 25)), ((-10, 25), (-10, -20))]


def raycast(x, y, ang, max_r=40.0):
    dx, dy = math.cos(ang), math.sin(ang)
    best = max_r
    for cx, cy, r in OBST:
        fx, fy = x - cx, y - cy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - r * r
        disc = b * b - c
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if 0.0 < t < best:
                best = t
    for (ax, ay), (bx, by) in WALLS:
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        t = ((ax - x) * ey - (ay - y) * ex) / den
        u = ((ax - x) * dy - (ay - y) * dx) / den
        if 0 < t < best and 0 <= u <= 1:
            best = t
    return best if best < max_r else float("inf")


def scan(x, y, yaw, rng, n=360, noise=0.02):
    az, el = beam_angles(n, 1, 2 * math.pi, 0.0)
    r = [raycast(x, y, yaw + a) for a in az]
    r = [v + rng.normal(0, noise) if math.isfinite(v) else v for v in r]
    return level_points(r, az, el, 0.0, 0.0, min_r=0.5, max_r=40.0, band=(-1, 1))


class TiltCompensation(unittest.TestCase):
    def test_level_wall_stays_level_when_tilted(self):
        """A vertical wall 10 m ahead must read ~10 m ahead, whatever roll/pitch the drone has."""
        az, el = beam_angles(1, 4, 0.0001, 0.3)
        for pitch in (0.0, 0.15, -0.2):
            # body-frame ray to the wall point (10, 0, z_w) seen from a drone pitched by `pitch`
            r = []
            for e in el:
                # level frame direction of this beam
                cp, sp = math.cos(pitch), math.sin(pitch)
                # beam in body frame -> level frame (what the code does)
                b = np.array([math.cos(e), 0.0, math.sin(e)])
                lv = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ b
                # distance along the beam until the level-frame x reaches 10 m
                r.append(10.0 / lv[0] if lv[0] > 1e-6 else float("inf"))
            hit, _ = level_points(r, az, el, 0.0, pitch, min_r=0.5, max_r=80.0, band=(-9, 9))
            self.assertTrue(np.allclose(hit[:, 0], 10.0, atol=1e-6))


class Slamming(unittest.TestCase):
    def run_path(self, noise_prior=0.05, steps=150):
        rng = np.random.default_rng(1)
        grid = OccupancyGrid(size_m=120, res=0.25)
        slam = Slam2D(grid, max_range=40.0)
        slam.reset(0.0, 0.0, 0.0, 0.0)
        x, y, yaw = 0.0, 0.0, 0.0
        errs = []
        t = 0.0
        for k in range(steps):
            t += 0.1
            x += 0.30                                 # 3 m/s ... straight then a gentle weave
            y = 3.0 * math.sin(x / 8.0)
            yaw = 0.3 * math.cos(x / 8.0)
            hits, miss = scan(x, y, yaw, rng)
            hint = (3.0 + rng.normal(0, 0.3), 0.0)        # imperfect velocity hint
            sx, sy, _, q = slam.update(t, hits, miss, yaw_imu=yaw + rng.normal(0, 0.003), vel_hint=hint)
            errs.append(math.hypot(sx - x, sy - y))
        return grid, errs, (x, y)

    def test_drift_is_small(self):
        _, errs, _ = self.run_path()
        self.assertLess(np.mean(errs), 0.35, f"mean error {np.mean(errs):.2f} m")
        self.assertLess(errs[-1], 0.8, f"final error {errs[-1]:.2f} m")

    def test_map_marks_obstacles(self):
        grid, _, _ = self.run_path()
        occ = grid.occupied()
        for cx, cy, r in OBST[:4]:                      # the ones the drone passed close to
            ij = grid.to_cell((cx - r, cy)) if False else grid.to_cell((cx, cy))
            # some occupied cell within r + 1 m of the true centre
            rad = int((r + 1.0) / grid.res)
            patch = occ[ij[0] - rad:ij[0] + rad + 1, ij[1] - rad:ij[1] + rad + 1]
            self.assertTrue(patch.any(), f"no occupied cell near obstacle at {(cx, cy)}")
        # and free space where the drone flew
        c = grid.to_cell((20.0, 3.0 * math.sin(20.0 / 8.0)))
        self.assertLess(grid.L[c[0], c[1]], 0.0)


class Planning(unittest.TestCase):
    def setUp(self):
        self.grid = OccupancyGrid(size_m=120, res=0.25)
        rng = np.random.default_rng(2)
        # build the map by flying straight through the world once (perfect poses)
        for x in np.arange(-5, 40, 1.0):
            h, m = scan(float(x), 0.0, 0.0, rng)
            self.grid.integrate(float(x), 0.0, 0.0, h, m)

    def test_path_avoids_inflated_obstacles(self):
        path = plan(self.grid, (-5.0, 4.0), (40.0, 4.0), radius_m=1.0)
        self.assertIsNotNone(path)
        pts = np.array(path)
        # sample densely along the path and check clearance from every obstacle
        for a, b in zip(pts[:-1], pts[1:]):
            for f in np.linspace(0, 1, 30):
                p = a + (b - a) * f
                for cx, cy, r in OBST:
                    self.assertGreater(math.hypot(p[0] - cx, p[1] - cy), r + 0.5,
                                       f"path clips obstacle {(cx, cy)} at {p}")
        self.assertLess(np.hypot(*(pts[-1] - (40, 4))), 1.0)

    def test_open_space_is_a_straight_line(self):
        path = plan(OccupancyGrid(size_m=120, res=0.25), (0, 0), (30, 20), radius_m=1.0)
        self.assertEqual(len(path), 2)

    def test_carrot_leads_along_path(self):
        c = carrot([(0, 0), (10, 0), (10, 10)], (2.0, 0.5), 5.0)
        self.assertAlmostEqual(c[0], 7.0, delta=0.2)
        self.assertAlmostEqual(c[1], 0.0, delta=0.2)
        c = carrot([(0, 0), (10, 0), (10, 10)], (9.0, 0.0), 5.0)
        self.assertAlmostEqual(c[0], 10.0, delta=0.2)
        self.assertAlmostEqual(c[1], 4.0, delta=0.3)


class Reactive(unittest.TestCase):
    def test_pushes_away_from_obstacle_ahead(self):
        fx, fy = obstacle_repulsion([(2.0, 0.0)], 0.0, 4.0)
        self.assertLess(fx, 0)                        # backwards
        self.assertGreater(abs(fy), 0)                # and sideways (tangential term)

    def test_world_frame_rotation(self):
        fx, fy = obstacle_repulsion([(2.0, 0.0)], math.pi, 4.0)   # facing -x: ahead is -x, push is +x
        self.assertGreater(fx, 0)

    def test_ignores_far_points_and_is_capped(self):
        self.assertEqual(obstacle_repulsion([(9.0, 0.0)], 0.0, 4.0), (0.0, 0.0))
        fx, fy = obstacle_repulsion([(0.6, 0.1 * k) for k in range(50)], 0.0, 4.0, cap=3.0)
        self.assertLessEqual(math.hypot(fx, fy), 3.0 + 1e-9)


class PeerMask(unittest.TestCase):
    def test_removes_points_near_a_peer_only(self):
        pts = np.array([[5.0, 0.0], [5.5, 0.5], [0.0, 10.0]])
        out = mask_peers(pts, 0.0, 0.0, 0.0, [(5.2, 0.2)], radius=1.8)
        self.assertEqual(out.tolist(), [[0.0, 10.0]])
        self.assertEqual(len(mask_peers(pts, 0.0, 0.0, 0.0, [], 1.8)), 3)


class ClosedLoop(unittest.TestCase):
    def test_flies_through_obstacle_field_using_only_slam_and_astar(self):
        """Point-mass drone, true kinematics, controller sees only lidar -> SLAM -> A* -> carrot."""
        rng = np.random.default_rng(3)
        nav = LeaderNav(size_m=120, res=0.25, radius=1.5, lookahead=5.0, replan_period=1.0)
        x, y, yaw, t = -5.0, 0.0, 0.0, 0.0
        nav.start(x, y, yaw, t)
        goal = (42.0, 4.0)
        worst = 1e9
        for _ in range(900):
            t += 0.1
            h, m = scan(x, y, yaw, rng)
            sx, sy, _, _ = nav.scan(t, h, m, yaw)
            tx, ty = nav.target(t, goal)
            dx, dy = tx - sx, ty - sy                          # steers by the SLAM pose...
            d = math.hypot(dx, dy)
            step = min(2.0 * 0.1, d)                           # ...but moves in the true world at 2 m/s
            if d > 1e-6:
                x += dx / d * step
                y += dy / d * step
            worst = min(worst, min(math.hypot(x - cx, y - cy) - r for cx, cy, r in OBST))
            if math.hypot(x - goal[0], y - goal[1]) < 1.5:
                break
        self.assertLess(math.hypot(x - goal[0], y - goal[1]), 1.5, f"stopped at {(x, y)}")
        self.assertGreater(worst, 0.3, f"came within {worst:.2f} m of an obstacle")


if __name__ == "__main__":
    unittest.main()
