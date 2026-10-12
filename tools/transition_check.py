"""Rough kinematic check of a formation change, no Webots needed.

Simulates the leader flying straight and N-1 followers using the SAME target and
repulsion logic as the controller, with a simple velocity-lag model instead of real
quadcopter dynamics. Reports the closest any two drones get during the change.

    python tools/transition_check.py --n 8 --layers 6 --from circle --to line
"""
import argparse
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "controllers" / "swarm_drone"))
from swarm_core import Peer, offsets, repulsion   # noqa: E402

ALT, STEP, TIMEOUT, LIM, TAU, DT = 10.0, 1.5, 1.0, 2.0, 0.7, 0.05


def run(n, layers, pat_from, pat_to, spacing=8.0, settle=60.0, window=90.0):
    alt = lambda i: ALT + STEP * ((i - 1) % layers)
    state = {i: [-6.0 * (i - 1), 0.0, alt(i), 0.0, 0.0, 0.0] for i in range(1, n + 1)}  # x y z vx vy vz
    t, worst, worst_pair, worst_t = 0.0, 1e9, None, None
    while t < settle + window:
        pattern = pat_from if t < settle else pat_to
        L = state[1]
        L[3], L[0] = 1.0, L[0] + 1.0 * DT                      # leader: 1 m/s along +x
        peers = {i: Peer(t, s[0], s[1], s[2], s[3], s[4], 0.0, 0, spacing) for i, s in state.items()}
        K = n - 1
        for i in range(2, n + 1):
            s = state[i]
            f, r = offsets(pattern, spacing, i - 2, K, t)
            tx, ty = L[0] + f, L[1] - r
            others = {j: p for j, p in peers.items() if j != i}
            rad = max(2.0, min(6.0, 0.75 * spacing))
            rx, ry = repulsion(i, s[0], s[1], s[2], others, t, TIMEOUT, rad)
            ex, ey = tx + rx - s[0], ty + ry - s[1]
            vdx, vdy = 0.8 * ex - 1.5 * s[3], 0.8 * ey - 1.5 * s[4]
            m = math.hypot(vdx, vdy)
            if m > LIM:
                vdx, vdy = vdx * LIM / m, vdy * LIM / m
            s[3] += (vdx - s[3]) * DT / TAU
            s[4] += (vdy - s[4]) * DT / TAU
            s[0] += s[3] * DT
            s[1] += s[4] * DT
            s[2] += (alt(i) - s[2]) * DT / TAU
        t += DT
        if t >= settle:
            for a in range(1, n + 1):
                for b in range(a + 1, n + 1):
                    d = math.dist(state[a][:3], state[b][:3])
                    if d < worst:
                        worst, worst_pair, worst_t = d, (a, b), t - settle
    return worst, worst_pair, worst_t


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=8)
    ap.add_argument("--layers", type=int, default=16)
    ap.add_argument("--from", dest="a", default="circle")
    ap.add_argument("--to", dest="b", default="line")
    ap.add_argument("--spacing", type=float, default=8.0)
    args = ap.parse_args()
    d, pair, tt = run(args.n, args.layers, args.a, args.b, args.spacing)
    print(f"n={args.n} layers={args.layers} {args.a}->{args.b}: closest approach "
          f"{d:.2f} m between drone{pair[0]} and drone{pair[1]} "
          f"({tt:.0f} s after the switch)")
