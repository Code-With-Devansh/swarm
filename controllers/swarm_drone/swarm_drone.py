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

from swarm_core import (PATTERNS, pack_state, unpack_state, roles, follower_slot,
                        offsets, repulsion, leader_speed_scale, slew_angle)

# ---------- configuration ----------
ALT = 10.0                     # base altitude (m)
LAYER_STEP = 1.5               # altitude gap between drones (m)
LAYERS = 16                    # one altitude per drone id, up to 16 drones
TIMEOUT = 1.0                  # s of silence before a drone counts as lost
TX_PERIOD = 0.1                # s between broadcasts (10 Hz)
DISCOVERY = 2.0                # s spent climbing in place, listening, before election
LEADER_VMAX = 0.5              # m/s: leader cruise speed (a real cap, not just a tilt limit)
FOLLOWER_VMAX = 3.0            # m/s: follower speed cap (in practice limited by tilt/drag)
FORM_RATE = 0.2                # rad/s: max rotation rate of the formation heading
WAIT_E0, WAIT_E1, WAIT_MIN = 4.0, 14.0, 0.15   # leader waits when a follower is >E0 m off its slot
CARROT_SPEED = 2.5             # m/s: max speed of the point a follower chases (see below)
TIP_LIMIT = 1.0                # rad: roll/pitch beyond this counts as tipped
TIP_FATAL = 1.5                # s tipped before the drone writes itself off
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
receiver = robot.getDevice("receiver")
if emitter is None or receiver is None:
    sys.exit(f"[drone{ID}] no emitter/receiver on this robot. Check that the world "
             "attaches them in a slot the Mavic2Pro PROTO accepts (bodySlot) "
             "-- see code/gen_world.py --slot")
receiver.enable(dt)
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
form_yaw = 0.0                    # formation heading the leader broadcasts (turns slowly)
slot_err = 0.0                    # my distance from my slot, broadcast so the leader can wait
leader_id = None                  # None during discovery
led_since = 0.0                   # sim time at which I last became leader
keys_prev = set()                 # keys down on the previous step (for edge detection)
failed = False
stall_since, stall_reported, tipped, tip_since = None, False, False, None
carrot = None                     # smoothed follower target (x, y)
last_cmd = None                   # last control commands, printed if I tip
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
            led_since = t
            form_yaw = yaw
            wi = min(range(len(PATROL)),
                     key=lambda k: math.hypot(PATROL[k][0] - x, PATROL[k][1] - y))
            leader_tyaw = yaw

    # ---- keyboard (every drone drains it, only the leader acts) ----
    # Webots reports a key on EVERY step while it is held, so act only on the
    # step where it goes down. Otherwise a held X would also fail the drone that
    # takes over a second later, and spacing would jump by hundreds of metres.
    keys_now = set()
    key = kb.getKey()
    while key > 0:
        keys_now.add(key & KEYMASK)
        key = kb.getKey()
    pressed, keys_prev = keys_now - keys_prev, keys_now

    if am_leader:
        for k in pressed:
            if ord("1") <= k <= ord("5"):
                pattern_idx = k - ord("1")
            elif k == ord("]"):
                spacing += 1
            elif k == ord("["):
                spacing = max(3.0, spacing - 1)
            elif k == ord("X") and t - led_since >= 3.0:   # new leaders get 3 s grace
                failed = True
                print(f"[drone{ID}] t={t:6.1f}s simulated FAILURE")

    if failed:                                # dead: silent, motors off
        for mo in motors:
            mo.setVelocity(0.0)
        continue

    # ---- pick a target ----
    tz, LIM, VMAX = MY_ALT, 2.0, FOLLOWER_VMAX
    slot_err, bc_yaw = 0.0, yaw
    if leader_id is None:                     # discovery: climb in place
        tx, ty, cmd_yaw, LIM, VMAX = sx, sy, yaw, 1.0, 1.0
    elif am_leader:
        wx, wy = PATROL[wi]
        dist = math.hypot(wx - x, wy - y)
        if dist < 2.5:
            wi = (wi + 1) % len(PATROL)
        if dist > 5:
            leader_tyaw = math.atan2(wy - y, wx - x)
        # wait for stragglers: slow down in proportion to the worst follower slot error
        worst = max((p.err for p in peers.values() if t - p.t <= TIMEOUT), default=0.0)
        VMAX = LEADER_VMAX * leader_speed_scale(worst, WAIT_E0, WAIT_E1, WAIT_MIN)
        # the formation heading turns slowly, so a leader corner doesn't whip the
        # whole formation around (the far slots would sweep tens of m/s)
        form_yaw = slew_angle(form_yaw, leader_tyaw, FORM_RATE * dt / 1000.0)
        tx, ty, cmd_yaw, LIM = wx, wy, leader_tyaw, 1.0
        bc_yaw = form_yaw
    else:
        L = peers.get(leader_id)
        if L is None or t - L.t > TIMEOUT:    # fixed-leader mode and leader silent
            tx, ty, cmd_yaw, LIM, VMAX = x, y, yaw, 1.0, 1.0    # hold position
        else:
            pattern_idx, spacing = L.pattern, L.spacing         # adopt leader's choice
            i, K = follower_slot(ID, followers)
            age = clamp(t - L.t, 0.0, TIMEOUT)
            lx, ly = L.x + L.vx * age, L.y + L.vy * age         # dead-reckon leader
            cmd_yaw = L.yaw                                     # formation heading
            f, r = offsets(PATTERNS[pattern_idx], spacing, i, K, t)
            tx = lx + f * math.cos(cmd_yaw) + r * math.sin(cmd_yaw)
            ty = ly + f * math.sin(cmd_yaw) - r * math.cos(cmd_yaw)
            slot_err = math.hypot(tx - x, ty - y)               # reported to the leader
            # Chase a "carrot" that creeps toward the slot at <= CARROT_SPEED instead of
            # the slot itself. A formation change or a leader turn can move a far slot by
            # tens of metres in an instant (or sweep it at tens of m/s); this keeps the
            # demand on the flight controller bounded.
            if carrot is None:
                carrot = (x, y)
            step = CARROT_SPEED * dt / 1000.0
            dx_, dy_ = tx - carrot[0], ty - carrot[1]
            dd = math.hypot(dx_, dy_)
            carrot = (tx, ty) if dd <= step else (carrot[0] + dx_ / dd * step,
                                                  carrot[1] + dy_ / dd * step)
            tx, ty = carrot

    if leader_id is None or am_leader:
        carrot = None                          # only followers use it; restart from where I am

    # ---- broadcast my state ----
    if t - last_tx >= TX_PERIOD:
        emitter.send(pack_state(ID, x, y, z, vx, vy, bc_yaw, pattern_idx, spacing, slot_err))
        last_tx = t

    # ---- collision avoidance: nudge the target away from close drones ----
    rx = ry = 0.0
    if leader_id is not None:
        radius = clamp(0.75 * spacing, 2.0, 6.0)
        rx, ry = repulsion(ID, x, y, z, peers, t, TIMEOUT, radius)
        tx += rx
        ty += ry

    # ---- flight controller ----
    # position error -> body frame (forward / left)
    ex, ey = tx - x, ty - y

    # ---- diagnostics: say so once if I stop short of my target or tip over ----
    if leader_id is not None and math.hypot(ex, ey) > 6.0 and math.hypot(vx, vy) < 0.15:
        stall_since = t if stall_since is None else stall_since
        if t - stall_since > 2.0 and not stall_reported:
            stall_reported = True
            near = min(((math.dist((x, y, z), (p.x, p.y, p.z)), pid)
                        for pid, p in peers.items() if t - p.t <= TIMEOUT),
                       default=(float("nan"), 0))
            print(f"[drone{ID}] t={t:6.1f}s STALLED at ({x:.1f},{y:.1f},{z:.1f}) "
                  f"{math.hypot(ex, ey):.1f} m from target, leader=drone{leader_id}, "
                  f"repulsion {math.hypot(rx, ry):.1f} m, nearest drone{near[1]} {near[0]:.1f} m")
    else:
        stall_since, stall_reported = None, False
    if abs(roll) > TIP_LIMIT or abs(pitch) > TIP_LIMIT:
        tip_since = t if tip_since is None else tip_since
        if not tipped:
            tipped = True
            print(f"[drone{ID}] t={t:6.1f}s TIPPED roll={roll:.2f} pitch={pitch:.2f} "
                  f"at ({x:.1f},{y:.1f},{z:.1f}) speed={math.hypot(vx, vy):.1f} m/s "
                  f"leader=drone{leader_id}")
            if last_cmd is not None:
                print("    last cmd: pitch_dist={:.2f} roll_dist={:.2f} yaw_dist={:.2f} "
                      "vertical={:.2f} | err fwd={:.1f} left={:.1f} | vel fwd={:.1f} left={:.1f} "
                      "| yaw={:.2f} target_yaw={:.2f} LIM={:.1f}".format(*last_cmd))
        if t - tip_since > TIP_FATAL:          # flipped and not recovering: go silent
            failed = True
            print(f"[drone{ID}] t={t:6.1f}s CRASHED, going silent so the others close up")
    else:
        tipped, tip_since = False, None

    c, s = math.cos(yaw), math.sin(yaw)
    ef, el = c * ex + s * ey, -s * ex + c * ey
    vf, vl = c * vx + s * vy, -s * vx + c * vy

    # desired velocity = 0.533 x error (same gain as before), but now with a real speed cap
    # VMAX; tilt demand = 1.5 x velocity error (identical to the old law below the cap)
    vdx, vdy = 0.5333 * ex, 0.5333 * ey
    vm = math.hypot(vdx, vdy)
    if vm > VMAX:
        vdx, vdy = vdx * VMAX / vm, vdy * VMAX / vm
    vdf, vdl = c * vdx + s * vdy, -s * vdx + c * vdy
    pitch_dist = -clamp(1.5 * (vdf - vf), -LIM, LIM)           # negative = forward
    roll_dist = clamp(1.5 * (vdl - vl), -LIM / 2, LIM / 2)     # positive = left
    yaw_dist = clamp(2.0 * wrap(cmd_yaw - yaw), -1.3, 1.3)
    vertical = 3.0 * clamp(tz - z + 0.6, -1.0, 1.0) ** 3

    roll_in = 50.0 * clamp(roll, -1, 1) + roll_rate + roll_dist
    pitch_in = 30.0 * clamp(pitch, -1, 1) + pitch_rate + pitch_dist
    base = 68.5 + vertical

    last_cmd = (pitch_dist, roll_dist, yaw_dist, vertical, ef, el, vf, vl, yaw, cmd_yaw, LIM)
    motors[0].setVelocity(base - roll_in + pitch_in - yaw_dist)    # front left
    motors[1].setVelocity(-(base + roll_in + pitch_in + yaw_dist)) # front right
    motors[2].setVelocity(-(base - roll_in - pitch_in + yaw_dist)) # rear left
    motors[3].setVelocity(base + roll_in - pitch_in - yaw_dist)    # rear right
