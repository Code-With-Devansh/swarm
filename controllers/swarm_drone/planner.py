"""A* on the (inflated) occupancy grid, plus path smoothing and a pure-pursuit carrot."""
import heapq
import math

import numpy as np


def _disk(r_cells):
    r = int(math.ceil(r_cells))
    return [(i, j) for i in range(-r, r + 1) for j in range(-r, r + 1) if i * i + j * j <= r_cells ** 2]


def inflate(occ, r_cells):
    out = np.zeros_like(occ, dtype=bool)
    idx = np.argwhere(occ)
    if len(idx) == 0:
        return out
    for di, dj in _disk(r_cells):
        i = np.clip(idx[:, 0] + di, 0, occ.shape[0] - 1)
        j = np.clip(idx[:, 1] + dj, 0, occ.shape[1] - 1)
        out[i, j] = True
    return out


def _nearest_free(blocked, cell, max_r=40):
    if not blocked[cell]:
        return cell
    for r in range(1, max_r):
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if max(abs(di), abs(dj)) != r:
                    continue
                i, j = cell[0] + di, cell[1] + dj
                if 0 <= i < blocked.shape[0] and 0 <= j < blocked.shape[1] and not blocked[i, j]:
                    return (i, j)
    return None


def _astar(blocked, soft, start, goal):
    h, w = blocked.shape
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    g = {start: 0.0}
    came = {}
    heap = [(0.0, start)]
    closed = set()
    while heap:
        _, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        closed.add(cur)
        for di, dj, c in moves:
            n = (cur[0] + di, cur[1] + dj)
            if not (0 <= n[0] < h and 0 <= n[1] < w) or blocked[n] or n in closed:
                continue
            if di and dj and (blocked[cur[0] + di, cur[1]] or blocked[cur[0], cur[1] + dj]):
                continue                                    # no cutting corners
            ng = g[cur] + c * (1.0 + 2.0 * soft[n])
            if ng < g.get(n, 1e18):
                g[n] = ng
                came[n] = cur
                dx, dy = abs(goal[0] - n[0]), abs(goal[1] - n[1])
                heapq.heappush(heap, (ng + (dx + dy) + (1.4142 - 2) * min(dx, dy), n))
    return None


def _clear(blocked, a, b):
    """True if the straight cell line a -> b crosses no blocked cell."""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
    for k in range(n + 1):
        f = k / max(n, 1)
        if blocked[int(round(a[0] + (b[0] - a[0]) * f)), int(round(a[1] + (b[1] - a[1]) * f))]:
            return False
    return True


def plan(grid, start_xy, goal_xy, radius_m, window_m=60.0, soft_m=2.0, occ_thresh=0.8):
    """Path from start to goal as a list of world (x, y), or None.

    Unknown cells count as free (the drone replans as it discovers obstacles).
    If the goal is outside the planning window, an intermediate goal on the window
    edge, towards the goal, is used.
    """
    res = grid.res
    s = grid.to_cell(start_xy)
    half = int(window_m / res)
    i0, j0 = max(int(s[0]) - half, 0), max(int(s[1]) - half, 0)
    i1, j1 = min(int(s[0]) + half, grid.n), min(int(s[1]) + half, grid.n)
    occ = grid.L[i0:i1, j0:j1] > occ_thresh
    blocked = inflate(occ, radius_m / res)
    soft = inflate(occ, (radius_m + soft_m) / res) & ~blocked
    org = np.array([i0, j0])

    sc = tuple(int(v) for v in (s - org))
    # never start boxed in by our own inflation
    for di, dj in _disk(radius_m / res):
        i, j = sc[0] + di, sc[1] + dj
        if 0 <= i < blocked.shape[0] and 0 <= j < blocked.shape[1]:
            blocked[i, j] = False
    gc = grid.to_cell(goal_xy) - org
    gc = np.array([int(np.clip(gc[0], 0, blocked.shape[0] - 1)), int(np.clip(gc[1], 0, blocked.shape[1] - 1))])
    g = _nearest_free(blocked, (int(gc[0]), int(gc[1])))
    if g is None:
        return None
    cells = _astar(blocked, soft, sc, g)
    if cells is None:
        return None
    # line-of-sight shortcutting
    out, k = [cells[0]], 0
    while k < len(cells) - 1:
        m = len(cells) - 1
        while m > k + 1 and not _clear(blocked, cells[k], cells[m]):
            m -= 1
        out.append(cells[m])
        k = m
    return [tuple(grid.to_world(np.array(c) + org)) for c in out]


def carrot(path, pos, lookahead):
    """Point `lookahead` metres along the path from the point nearest to `pos`."""
    if not path:
        return None
    if len(path) == 1:
        return path[0]
    best, bd = 0, 1e18
    for k in range(len(path) - 1):
        ax, ay = path[k]
        bx, by = path[k + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy or 1e-9
        u = max(0.0, min(1.0, ((pos[0] - ax) * dx + (pos[1] - ay) * dy) / L2))
        d = math.hypot(ax + u * dx - pos[0], ay + u * dy - pos[1])
        if d < bd:
            best, bd, bu = k, d, u
    k, u = best, bu
    ax, ay = path[k]
    bx, by = path[k + 1]
    cx, cy = ax + u * (bx - ax), ay + u * (by - ay)
    remain = lookahead
    while True:
        seg = math.hypot(bx - cx, by - cy)
        if seg >= remain:
            f = remain / max(seg, 1e-9)
            return (cx + (bx - cx) * f, cy + (by - cy) * f)
        remain -= seg
        k += 1
        cx, cy = bx, by
        if k >= len(path) - 1:
            return path[-1]
        bx, by = path[k + 1]
