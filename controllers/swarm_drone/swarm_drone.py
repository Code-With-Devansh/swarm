"""Decentralised swarm drone controller (Webots, Mavic 2 Pro).

Every drone runs this same file. There is no supervisor access between drones:
they only know what arrives over the Emitter/Receiver radio (10 Hz state
broadcasts) plus their own GPS / IMU / gyro.

  * Leader election: the lowest-id drone that is still transmitting leads.
    If the leader goes silent for TIMEOUT seconds, the next one takes over.
    (controllerArgs ["fixed"] disables this: drone1 is always leader.)
  * Leader: patrols a square and reads the keyboard.
  * Followers: fly to their slot in the leader's formation.
  * Everyone: potential-field repulsion from nearby drones.

Keyboard (focus the 3D view; only the current leader reacts):
    1..5   line / v / grid / circle / square
    ]  [   spacing +1 / -1 metres (min 3)
    X      simulate failure of the current leader (its motors cut out)
"""
import math
import sys

from controller import Robot, Keyboard

from swarm_core import (PATTERNS, pack_state, unpack_state, roles,
                        follower_slot, offsets, repulsion)

# ---------- configuration ----------
ALT = 10.0                     # base altitude (m)
LAYER_STEP = 1.5               # altitude gap between drones (m)
LAYERS = 6                     # distinct altitude layers before wrapping
TIMEOUT = 1.0                  # s of silence before a drone counts as lost
TX_PERIOD = 0.1                # s between broadcasts (10 Hz)
DISCOVERY = 2.0                # s spent climbing in place, listening, before election
FAILOVER = "fixed" not in sys.argv[1:]
PATROL = [(40.0, 0.0), (40.0, 40.0), (0.0, 40.0), (0.0, 0.0)]   # world frame

robot = Robot()
dt = int(robot.getBasicTimeStep())
ID = int(robot.getName().replace("drone", ""))
MY_ALT = ALT + LAYER_STEP * ((ID - 1) % LAYERS)

# ---------- devices ----------
imu = robot.getDevice("inertial unit"); imu.enable(dt)
gps = robot.getDevice("gps");           gps.enable(dt)
gyro = robot.getDevice("gyro");         gyro.enable(dt)
motors = [robot.getDevice(n + " propeller")
          for n in ("front left", "front right", "rear left", "rear right")]
for mo in motors:
    mo.setPosition(float("inf"))
    mo.setVelocity(1.0)

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver"); receiver.enable(dt)
kb = robot.getKeyboard(); kb.enable(dt)
KEYMASK = getattr(Keyboard, "KEY", 0xFFFF)


def clamp(v, lo, hi): return max(lo, min(hi, v))
def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi


peers = {}          # drone id -> Peer (last packet heard)


def listen(t):
    while receiver.getQueueLength() > 0:
        try:
            data = receiver.getBytes()
        except AttributeError:               # older Webots
            data = receiver.getData()
        receiver.nextPacket()
        msg = unpack_state(data, t)
        if msg is not None and msg[0] != ID:
            peers[msg[0]] = msg[1]


# let sensors warm up
while robot.step(dt) != -1:
    if robot.getTime() > 1.0:
        break
sx, sy, _ = gps.getValues()

# ---------- state ----------
pattern_idx, spacing = 0, 8.0     # leader sets these, followers copy them
wi, leader_tyaw = 0, 0.0          # waypoint index / heading target while leading
leader_id = None                  # None during discovery
failed = False
last_tx = -1e9
discovery_end = robot.getTime() + DISCOVERY

# ---------- main loop ----------
while robot.step(dt) != -1:
    t = robot.getTime()
    x, y, z = gps.getValues()
    vx, vy, _ = gps.getSpeedVector()
    roll, pitch, yaw = imu.getRollPitchYaw()
    roll_rate, pitch_rate, _ = gyro.getValues()

    listen(t)

    # ---- who leads? (computed locally from what I have heard) ----
    prev_leader = leader_id
    if t < discovery_end:
        leader_id, followers = None, []
    else:
        leader_id, followers = roles(ID, peers, t, TIMEOUT,
                                     None if FAILOVER else 1)
    am_leader = leader_id == ID

    if leader_id != prev_leader and leader_id is not None:
        print(f"[drone{ID}] t={t:6.1f}s leader is drone{leader_id}"
              f"{' (me)' if am_leader else ''}")
        if am_leader:                         # take over: resume patrol from here
            wi = min(range(len(PATROL)),
                     key=lambda k: math.hypot(PATROL[k][0] - x, PATROL[k][1] - y))
            leader_tyaw = yaw

    # ---- keyboard (every drone drains it, only the leader acts) ----
    key = kb.getKey()
    while key > 0:
        k = key & KEYMASK
        if am_leader:
            if ord("1") <= k <= ord("5"):
                pattern_idx = k - ord("1")
            elif k == ord("]"):
                spacing += 1
            elif k == ord("["):
                spacing = max(3.0, spacing - 1)
            elif k == ord("X"):
                failed = True
                print(f"[drone{ID}] t={t:6.1f}s simulated FAILURE")
        key = kb.getKey()

    if failed:                                # dead: silent, motors off
        for mo in motors:
            mo.setVelocity(0.0)
        continue

    # ---- broadcast my state ----
    if t - last_tx >= TX_PERIOD:
        emitter.send(pack_state(ID, x, y, z, vx, vy, yaw, pattern_idx, spacing))
        last_tx = t

    # ---- pick a target ----
    tz, LIM = MY_ALT, 2.0
    if leader_id is None:                     # discovery: climb in place
        tx, ty, cmd_yaw, LIM = sx, sy, yaw, 1.0
    elif am_leader:
        wx, wy = PATROL[wi]
        dist = math.hypot(wx - x, wy - y)
        if dist < 2.5:
            wi = (wi + 1) % len(PATROL)
        if dist > 5:
            leader_tyaw = math.atan2(wy - y, wx - x)
        tx, ty, cmd_yaw, LIM = wx, wy, leader_tyaw, 1.0       # leader flies slowly
    else:
        L = peers.get(leader_id)
        if L is None or t - L.t > TIMEOUT:    # fixed-leader mode and leader silent
            tx, ty, cmd_yaw, LIM = x, y, yaw, 1.0               # hold position
        else:
            pattern_idx, spacing = L.pattern, L.spacing         # adopt leader's choice
            i, K = follower_slot(ID, followers)
            age = clamp(t - L.t, 0.0, TIMEOUT)
            lx, ly = L.x + L.vx * age, L.y + L.vy * age         # dead-reckon leader
            cmd_yaw = L.yaw
            f, r = offsets(PATTERNS[pattern_idx], spacing, i, K, t)
            tx = lx + f * math.cos(cmd_yaw) + r * math.sin(cmd_yaw)
            ty = ly + f * math.sin(cmd_yaw) - r * math.cos(cmd_yaw)

    # ---- collision avoidance: nudge the target away from close drones ----
    if leader_id is not None:
        radius = clamp(0.75 * spacing, 2.0, 6.0)
        rx, ry = repulsion(ID, x, y, z, peers, t, TIMEOUT, radius)
        tx += rx
        ty += ry

    # ---- flight controller (unchanged from the original) ----
    # position error -> body frame (forward / left)
    ex, ey = tx - x, ty - y
    c, s = math.cos(yaw), math.sin(yaw)
    ef, el = c * ex + s * ey, -s * ex + c * ey
    vf, vl = c * vx + s * vy, -s * vx + c * vy

    pitch_dist = -clamp(0.8 * ef - 1.5 * vf, -LIM, LIM)        # negative = forward
    roll_dist = clamp(0.8 * el - 1.5 * vl, -LIM / 2, LIM / 2)  # positive = left
    yaw_dist = clamp(2.0 * wrap(cmd_yaw - yaw), -1.3, 1.3)
    vertical = 3.0 * clamp(tz - z + 0.6, -1.0, 1.0) ** 3

    roll_in = 50.0 * clamp(roll, -1, 1) + roll_rate + roll_dist
    pitch_in = 30.0 * clamp(pitch, -1, 1) + pitch_rate + pitch_dist
    base = 68.5 + vertical

    motors[0].setVelocity(base - roll_in + pitch_in - yaw_dist)    # front left
    motors[1].setVelocity(-(base + roll_in + pitch_in + yaw_dist)) # front right
    motors[2].setVelocity(-(base - roll_in - pitch_in + yaw_dist)) # rear left
    motors[3].setVelocity(base + roll_in - pitch_in - yaw_dist)    # rear right
