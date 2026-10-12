"""Monitor: logs minimum drone separation and keeps the camera on the leader.

The only supervisor in the world. It also has a Receiver, so it hears the same
10 Hz broadcasts as the drones and works out the leader the same way they do
(lowest id still transmitting). When the leader changes, it points the
Viewpoint's `follow` field at the new leader, so the camera no longer stays
locked to a crashed drone1.

controllerArgs: ["<number of drones>"] or ["<number of drones>", "fixed"].
The world's Viewpoint must be `DEF VIEWPOINT Viewpoint {...}` (gen_world.py --inject does this).
"""
import math
import os
import sys
from itertools import combinations

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "swarm_drone"))

from controller import Supervisor
from swarm_core import unpack_state

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
args = sys.argv[1:]
N = next((int(a) for a in args if a.isdigit()), 5)
FIXED = "fixed" in args
WARN_DIST = 2.0        # m
MIN_FLIGHT_Z = 3.0     # ignore drones still on the ground / crashed
PERIOD = 1.0           # s between log lines
TIMEOUT = 1.0          # s of silence before a drone counts as lost (same as the drones)

nodes = {i: robot.getFromDef(f"DRONE{i}") for i in range(1, N + 1)}
nodes = {i: n for i, n in nodes.items() if n is not None}

receiver = robot.getDevice("receiver")
if receiver is not None:
    receiver.enable(dt)
viewpoint = robot.getFromDef("VIEWPOINT")
follow = viewpoint.getField("follow") if viewpoint is not None else None
if follow is None:
    print("[monitor] no DEF VIEWPOINT in the world: camera will not follow the leader")

heard = {}             # drone id -> time last heard
cam_target = None
worst = float("inf")
next_log = 0.0
while robot.step(dt) != -1:
    t = robot.getTime()

    # ---- camera: follow the current leader ----
    if receiver is not None:
        while receiver.getQueueLength() > 0:
            try:
                data = receiver.getBytes()
            except AttributeError:
                data = receiver.getData()
            receiver.nextPacket()
            msg = unpack_state(data, t)
            if msg is not None:
                heard[msg[0]] = t
    live = [i for i, th in heard.items() if t - th <= TIMEOUT]
    leader = 1 if FIXED else (min(live) if live else None)
    if follow is not None and leader is not None and leader != cam_target:
        follow.setSFString(f"drone{leader}")
        cam_target = leader
        print(f"[monitor] t={t:6.1f}s camera now follows drone{leader}")

    # ---- separation log ----
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
