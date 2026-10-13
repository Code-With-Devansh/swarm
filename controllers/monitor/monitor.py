"""Monitor / demo helper. The only supervisor in the world.

  * Camera: press V (3D view focused) to cycle   overview -> chase leader -> top-down.
    Chase re-targets itself when the leader changes.
  * Beacons: the glowing ball over each drone is gold for the leader, cyan for followers
    and red for a drone that has gone silent (crashed / failed).
  * Safety log: once a second, the minimum separation between flying drones.

It listens to the same radio broadcasts as the drones, so it knows the leader the same
way they do (lowest id still transmitting).

controllerArgs: ["<number of drones>"] or ["<number of drones>", "fixed"].
The world's Viewpoint must be `DEF VIEWPOINT Viewpoint {...}` (gen_world.py --inject does this).
"""
import math
import os
import sys
from itertools import combinations

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "swarm_drone"))

from controller import Supervisor, Keyboard
from swarm_core import unpack_state, look_at

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
args = sys.argv[1:]
N = next((int(a) for a in args if a.isdigit()), 5)
FIXED = "fixed" in args
WARN_DIST = 2.0        # m
MIN_FLIGHT_Z = 3.0     # ignore drones still on the ground / crashed
PERIOD = 1.0           # s between log lines
TIMEOUT = 1.0          # s of silence before a drone counts as lost (same as the drones)

# ---- camera presets (world frame, metres) ----
VIEW_OVERVIEW, VIEW_CHASE, VIEW_TOP = 0, 1, 2
VIEW_NAMES = ["overview", "chase leader", "top-down"]
START_VIEW = VIEW_OVERVIEW
OVERVIEW = dict(pos=(20.0, -120.0, 60.0), target=(20.0, 15.0, 12.0), fov=0.6)
TOP = dict(pos=(15.0, 15.0, 130.0), target=(15.0, 15.0, 0.0), fov=0.75)
CHASE_OFFSET = (-28.0, -28.0, 18.0)       # camera position relative to the leader
CHASE_FOV = 0.75

GOLD, CYAN, RED = [1.0, 0.8, 0.1], [0.1, 0.7, 1.0], [1.0, 0.1, 0.1]

nodes = {i: robot.getFromDef(f"DRONE{i}") for i in range(1, N + 1)}
nodes = {i: n for i, n in nodes.items() if n is not None}
beacons = {}
for i in nodes:
    b = robot.getFromDef(f"BEACON{i}")
    if b is not None:
        beacons[i] = (b.getField("baseColor"), b.getField("emissiveColor"))

receiver = robot.getDevice("receiver")
if receiver is not None:
    receiver.enable(dt)
kb = robot.getKeyboard()
kb.enable(dt)
KEYMASK = getattr(Keyboard, "KEY", 0xFFFF)

viewpoint = robot.getFromDef("VIEWPOINT")
if viewpoint is None:
    print("[monitor] no DEF VIEWPOINT in the world: camera control disabled")
    f_follow = f_pos = f_ori = f_fov = None
else:
    f_follow, f_pos = viewpoint.getField("follow"), viewpoint.getField("position")
    f_ori, f_fov = viewpoint.getField("orientation"), viewpoint.getField("fieldOfView")


def set_pose(pos, target, fov):
    f_follow.setSFString("")
    f_pos.setSFVec3f(list(pos))
    f_ori.setSFRotation(list(look_at(pos, target)))
    f_fov.setSFFloat(fov)


def apply_view(mode, leader):
    if viewpoint is None:
        return
    if mode == VIEW_OVERVIEW:
        set_pose(OVERVIEW["pos"], OVERVIEW["target"], OVERVIEW["fov"])
    elif mode == VIEW_TOP:
        set_pose(TOP["pos"], TOP["target"], TOP["fov"])
    else:                                       # chase: place the camera, then lock onto the leader
        node = nodes.get(leader)
        if node is None:
            return
        lp = node.getPosition()
        cam = [lp[0] + CHASE_OFFSET[0], lp[1] + CHASE_OFFSET[1], lp[2] + CHASE_OFFSET[2]]
        set_pose(cam, lp, CHASE_FOV)
        f_follow.setSFString(f"drone{leader}")


def recolor(leader, live, t):
    for i, (base, emis) in beacons.items():
        if i == leader and i in live:
            col = GOLD
        elif i in live:
            col = CYAN
        elif t > 3.0:
            col = RED
        else:
            continue
        base.setSFColor(col)
        emis.setSFColor(col)


heard = {}             # drone id -> time last heard
view = START_VIEW
view_dirty = True      # apply the starting view on the first step
cam_leader = None
colors_key = None
keys_prev = set()
worst = float("inf")
next_log = 0.0
print("[monitor] camera: press V to cycle views (overview / chase leader / top-down)")
while robot.step(dt) != -1:
    t = robot.getTime()

    # ---- radio: who is alive, who leads ----
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
    live = {i for i, th in heard.items() if t - th <= TIMEOUT}
    leader = 1 if FIXED else (min(live) if live else None)

    # ---- keyboard: V cycles the camera view ----
    keys_now = set()
    key = kb.getKey()
    while key > 0:
        keys_now.add(key & KEYMASK)
        key = kb.getKey()
    if ord("V") in keys_now - keys_prev:
        view = (view + 1) % len(VIEW_NAMES)
        view_dirty = True
        print(f"[monitor] camera view: {VIEW_NAMES[view]}")
    keys_prev = keys_now

    # ---- camera ----
    if leader != cam_leader:
        if leader is not None:
            print(f"[monitor] t={t:6.1f}s leader is drone{leader}")
        if view == VIEW_CHASE:
            view_dirty = True                   # chase follows leader changes
        cam_leader = leader
    if view_dirty and (view != VIEW_CHASE or leader is not None):
        apply_view(view, leader)
        view_dirty = False

    # ---- beacons ----
    key_now = (leader, frozenset(live), t > 3.0)
    if key_now != colors_key:
        recolor(leader, live, t)
        colors_key = key_now

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
