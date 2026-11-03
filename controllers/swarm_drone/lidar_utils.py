"""Lidar helpers: turn a Webots range image into level, body-frame 2D points.

No Webots imports. numpy only.

Frames
  body   : attached to the drone, x forward, y left, z up (Webots ENU)
  level  : body frame de-rotated by roll and pitch (yaw is kept), so a
           horizontal wall stays horizontal while the quadcopter tilts to accelerate

Body -> world is Rz(yaw) Ry(pitch) Rx(roll), so level = Ry(pitch) Rx(roll) body.
"""
import math

import numpy as np


def beam_angles(h_res, layers, fov_h, fov_v, top_first=True, first_left=True):
    """(azimuth[h_res], elevation[layers]) in radians, in the sensor frame.

    Webots orders the range image layer by layer. Whether layer 0 is the top one and
    whether column 0 is the leftmost ray are the two things to VERIFY once on a real
    run (point the drone at a known wall); flip `top_first` / `first_left` if the map
    comes out mirrored.
    """
    j = np.arange(h_res)
    az = (fov_h / 2 - (j + 0.5) * fov_h / h_res) if first_left else (-fov_h / 2 + (j + 0.5) * fov_h / h_res)
    if layers == 1:
        el = np.zeros(1)
    else:
        el = np.linspace(fov_v / 2, -fov_v / 2, layers) if top_first else np.linspace(-fov_v / 2, fov_v / 2, layers)
    return az, el


def level_points(ranges, az, el, roll, pitch, mount=(0.0, 0.0, 0.0),
                 min_r=0.5, max_r=40.0, band=(-1.5, 1.5)):
    """Range image -> (pts_xy, hit_xy, miss_xy) in the level frame.

    ranges : flat sequence, layer-major (len = layers * len(az))
    returns:
      pts_xy  (N,2) endpoints of beams that hit something, inside the height band
      miss_xy (M,2) endpoints at max_r for beams that returned nothing (free-space only)
    """
    r = np.asarray(ranges, dtype=np.float64).reshape(len(el), len(az))
    ce, se = np.cos(el)[:, None], np.sin(el)[:, None]
    ca, sa = np.cos(az)[None, :], np.sin(az)[None, :]
    finite = np.isfinite(r) & (r >= min_r) & (r < max_r - 1e-6)
    nothing = ~np.isfinite(r) | (r >= max_r - 1e-6)

    def to_level(rr, mask):
        x = (rr * ce * ca)[mask] + mount[0]
        y = (rr * ce * sa)[mask] + mount[1]
        z = (rr * se * np.ones_like(ca))[mask] + mount[2]
        p = np.stack([x, y, z], axis=1)
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        return p @ (ry @ rx).T

    hit = to_level(np.where(finite, r, 0.0), finite)
    hit = hit[(hit[:, 2] >= band[0]) & (hit[:, 2] <= band[1])][:, :2]

    # beams with no return: free space up to max_r, only for the near-horizontal ones
    far = np.where(nothing, max_r, 0.0)
    miss = to_level(far, nothing)
    miss = miss[(np.abs(miss[:, 2]) <= abs(band[1]) * 4)][:, :2]
    return hit, miss


def mask_peers(pts, x, y, yaw, peers_xy, radius=1.8):
    """Drop level-frame points that lie within `radius` m of another drone.

    Other drones in the lidar's height band would otherwise be mapped as (moving)
    walls and wreck both the map and the scan matching. `peers_xy` are world positions
    predicted from the radio packets.
    """
    if len(pts) == 0 or not peers_xy:
        return pts
    c, s = math.cos(yaw), math.sin(yaw)
    w = pts @ np.array([[c, s], [-s, c]]) + (x, y)
    keep = np.ones(len(pts), dtype=bool)
    for px, py in peers_xy:
        keep &= np.hypot(w[:, 0] - px, w[:, 1] - py) > radius
    return pts[keep]
