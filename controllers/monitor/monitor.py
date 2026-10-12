"""Safety monitor: logs the minimum pairwise distance between flying drones.

This is the only supervisor in the world; the drones themselves no longer
have supervisor access. Usage: controllerArgs ["5"] (number of drones).
"""
import math
import sys
from itertools import combinations

from controller import Supervisor

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
N = int(sys.argv[1]) if len(sys.argv) > 1 else 5
WARN_DIST = 2.0        # m
MIN_FLIGHT_Z = 3.0     # ignore drones still on the ground / crashed
PERIOD = 1.0           # s between log lines

nodes = {i: robot.getFromDef(f"DRONE{i}") for i in range(1, N + 1)}
nodes = {i: n for i, n in nodes.items() if n is not None}

worst = float("inf")
next_log = 0.0
while robot.step(dt) != -1:
    t = robot.getTime()
    if t < next_log:
        continue
    next_log = t + PERIOD

    pos = {i: n.getPosition() for i, n in nodes.items()}
    flying = {i: p for i, p in pos.items() if p[2] > MIN_FLIGHT_Z}
    if len(flying) < 2:
        continue

    d, a, b = min((math.dist(flying[a], flying[b]), a, b)
                  for a, b in combinations(flying, 2))
    worst = min(worst, d)
    flag = "  <-- TOO CLOSE" if d < WARN_DIST else ""
    print(f"[monitor] t={t:6.1f}s  min separation {d:5.2f} m "
          f"(drone{a}-drone{b})  worst so far {worst:5.2f} m{flag}")
