"""Pure swarm logic: no Webots imports, so it can be unit tested anywhere.

Contains
  * the radio message format (pack_state / unpack_state)
  * leader election and follower slot assignment (roles / follower_slot)
  * formation slot offsets (offsets)
  * collision-avoidance repulsion (repulsion)
"""
import math
import struct
from collections import namedtuple

PATTERNS = ["line", "v", "grid", "circle", "square"]
CIRCLE_RATE = 0.05              # rad/s, orbit speed of the circle formation (~0.75 m/s at r=15)

# id, x, y, z, vx, vy, yaw, pattern index, spacing, slot error   (little endian, no padding)
#   yaw        : for the leader this is the (slowly turning) FORMATION heading
#   slot error : how far a follower is from its slot, in metres (0 for the leader)
MSG_FMT = "<B6fBff"
MSG_SIZE = struct.calcsize(MSG_FMT)

# t = simulation time at which the packet was RECEIVED
Peer = namedtuple("Peer", "t x y z vx vy yaw pattern spacing err", defaults=(0.0,))


# --------------------------------------------------------------------------
# Messages
# --------------------------------------------------------------------------
def pack_state(drone_id, x, y, z, vx, vy, yaw, pattern_idx, spacing, err=0.0):
    return struct.pack(MSG_FMT, drone_id, x, y, z, vx, vy, yaw, pattern_idx, spacing, err)


def unpack_state(data, t):
    """Return (drone_id, Peer) or None if the packet is malformed."""
    if len(data) != MSG_SIZE:
        return None
    pid, x, y, z, vx, vy, yaw, pat, sp, err = struct.unpack(MSG_FMT, data)
    if pat >= len(PATTERNS):
        return None
    if not all(math.isfinite(v) for v in (x, y, z, vx, vy, yaw, sp, err)):
        return None
    return pid, Peer(t, x, y, z, vx, vy, yaw, pat, sp, err)


# --------------------------------------------------------------------------
# Roles: every drone runs this on its own copy of `peers`, no negotiation.
# --------------------------------------------------------------------------
def live_ids(self_id, peers, t, timeout):
    """Sorted ids of all drones heard from within `timeout`, plus myself."""
    return sorted({self_id} | {pid for pid, p in peers.items() if t - p.t <= timeout})


def roles(self_id, peers, t, timeout, fixed_leader=None):
    """Return (leader_id, followers) where followers is a sorted id list.

    Failover mode (fixed_leader=None): the lowest live id leads.
    Fixed mode: `fixed_leader` always leads, even if it has gone silent.
    """
    live = live_ids(self_id, peers, t, timeout)
    leader = live[0] if fixed_leader is None else fixed_leader
    followers = [d for d in live if d != leader]
    return leader, followers


def follower_slot(self_id, followers):
    """(slot index i, follower count k) for this drone."""
    return followers.index(self_id), len(followers)


# --------------------------------------------------------------------------
# Formations: (forward, right) in metres in the leader's frame
# --------------------------------------------------------------------------
def offsets(pattern, sp, i, k, t):
    if pattern == "line":
        return (-sp * (i + 1), 0)
    if pattern == "v":
        row, side = i // 2 + 1, (-1 if i % 2 == 0 else 1)
        return (-sp * row, side * sp * row)
    if pattern == "grid":
        row, col = i // 3 + 1, i % 3 - 1
        return (-sp * row, sp * col)
    if pattern == "circle":
        r = max(15, sp * k / (2 * math.pi) * 1.5)
        # keep the orbit speed well inside what followers can fly (the original 0.2 rad/s
        # asked for 3 m/s at r=15, so they could never keep their slots)
        a = 2 * math.pi * i / max(k, 1) + CIRCLE_RATE * t
        return (r * math.cos(a), r * math.sin(a))
    if pattern == "square":
        h = max(10, sp * k / 8)
        u = (i / max(k, 1)) * 4
        side, f = int(u), u - int(u)
        if side == 0:
            return (h, -h + 2 * h * f)
        if side == 1:
            return (h - 2 * h * f, h)
        if side == 2:
            return (-h, h - 2 * h * f)
        return (-h + 2 * h * f, -h)
    return (-sp * (i + 1), 0)


# --------------------------------------------------------------------------
# Collision avoidance
# --------------------------------------------------------------------------
def predict(p, t, max_age):
    """Dead-reckon a peer's position forward to time t."""
    age = max(0.0, min(t - p.t, max_age))
    return p.x + p.vx * age, p.y + p.vy * age, p.z


def repulsion(self_id, x, y, z, peers, t, timeout, radius,
              k=4.0, cap=4.0, tangent=0.35):
    """Horizontal target offset (fx, fy) pushing me away from close peers.

    * distance counts vertical gaps double, so drones in different altitude
      layers barely repel each other
    * a small tangential term makes every drone swerve the same way
      (counter-clockwise), which breaks head-on symmetry deadlocks
    * result is capped at `cap` metres
    """
    fx = fy = 0.0
    for pid, p in peers.items():
        if t - p.t > timeout:
            continue
        px, py, pz = predict(p, t, timeout)
        dx, dy, dz = x - px, y - py, z - pz
        d = math.sqrt(dx * dx + dy * dy + (2.0 * dz) ** 2)
        if d >= radius:
            continue
        h = math.hypot(dx, dy)
        if h < 1e-3:                       # stacked vertically: pick a side by id
            ux, uy = (1.0, 0.0) if self_id > pid else (-1.0, 0.0)
        else:
            ux, uy = dx / h, dy / h
        mag = k * (1.0 / max(d, 0.5) - 1.0 / radius)
        fx += mag * (ux - tangent * uy)
        fy += mag * (uy + tangent * ux)
    m = math.hypot(fx, fy)
    if m > cap:
        fx, fy = fx * cap / m, fy * cap / m
    return fx, fy


# --------------------------------------------------------------------------
# Keeping the swarm together
# --------------------------------------------------------------------------
def leader_speed_scale(worst_err, e0, e1, floor):
    """Fraction of cruise speed the leader may fly, given the worst follower slot error.

    1.0 while everyone is within e0 metres of their slot, easing down to `floor`
    at e1 metres, so the leader waits for stragglers instead of outrunning them.
    """
    if e1 <= e0:
        return 1.0
    return max(floor, min(1.0, 1.0 - (worst_err - e0) / (e1 - e0)))


def slew_angle(current, target, max_step):
    """Turn `current` toward `target` by at most max_step radians (shortest way round)."""
    d = (target - current + math.pi) % (2 * math.pi) - math.pi
    return current + max(-max_step, min(max_step, d))


# --------------------------------------------------------------------------
# Camera helper (used by the monitor)
# --------------------------------------------------------------------------
def look_at(pos, target, up=(0.0, 0.0, 1.0)):
    """Webots Viewpoint `orientation` (axis x, y, z, angle) that looks from pos at target.

    A Webots (ENU) viewpoint looks along its local +x axis with local +z up, which is
    what the project's own world file uses.
    """
    f = [target[i] - pos[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in f)) or 1.0
    f = [c / n for c in f]
    left = [up[1] * f[2] - up[2] * f[1], up[2] * f[0] - up[0] * f[2], up[0] * f[1] - up[1] * f[0]]
    n = math.sqrt(sum(c * c for c in left))
    if n < 1e-6:                               # looking straight along `up`: pick another up
        return look_at(pos, target, (0.0, 1.0, 0.0))
    left = [c / n for c in left]
    u = [f[1] * left[2] - f[2] * left[1], f[2] * left[0] - f[0] * left[2], f[0] * left[1] - f[1] * left[0]]
    # rotation matrix with columns f, left, u  ->  quaternion  ->  axis/angle
    m = [[f[0], left[0], u[0]], [f[1], left[1], u[1]], [f[2], left[2], u[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        sq = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * sq, (m[2][1] - m[1][2]) / sq, (m[0][2] - m[2][0]) / sq, (m[1][0] - m[0][1]) / sq
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        sq = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w, x, y, z = (m[2][1] - m[1][2]) / sq, 0.25 * sq, (m[0][1] + m[1][0]) / sq, (m[0][2] + m[2][0]) / sq
    elif m[1][1] > m[2][2]:
        sq = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w, x, y, z = (m[0][2] - m[2][0]) / sq, (m[0][1] + m[1][0]) / sq, 0.25 * sq, (m[1][2] + m[2][1]) / sq
    else:
        sq = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w, x, y, z = (m[1][0] - m[0][1]) / sq, (m[0][2] + m[2][0]) / sq, (m[1][2] + m[2][1]) / sq, 0.25 * sq
    angle = 2 * math.acos(max(-1.0, min(1.0, w)))
    s_ = math.sqrt(max(0.0, 1.0 - w * w))
    if s_ < 1e-9:
        return (0.0, 0.0, 1.0, 0.0)
    return (x / s_, y / s_, z / s_, angle)
