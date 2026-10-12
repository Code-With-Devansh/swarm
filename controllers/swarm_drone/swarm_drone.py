import math, sys
from controller import Supervisor, Keyboard

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
ID = int(robot.getName().replace("drone", ""))
N = int(sys.argv[1]) if len(sys.argv) > 1 else 5
K = N - 1                      # number of followers
LEADER = (ID == 1)
ALT = 10.0

# ---------- devices ----------
imu = robot.getDevice("inertial unit"); imu.enable(dt)
gps = robot.getDevice("gps");           gps.enable(dt)
gyro = robot.getDevice("gyro");         gyro.enable(dt)
motors = [robot.getDevice(n + " propeller")
          for n in ("front left", "front right", "rear left", "rear right")]
for mo in motors:
    mo.setPosition(float("inf"))
    mo.setVelocity(1.0)

def clamp(v, lo, hi): return max(lo, min(hi, v))
def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi

# ---------- formations: (forward, right) in metres, leader's frame ----------
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
        a = 2 * math.pi * i / max(k, 1) + 0.2 * t
        return (r * math.cos(a), r * math.sin(a))
    if pattern == "square":
        h = max(10, sp * k / 8)
        u = (i / max(k, 1)) * 4
        side, f = int(u), u - int(u)
        if side == 0: return (h, -h + 2 * h * f)
        if side == 1: return (h - 2 * h * f, h)
        if side == 2: return (-h, h - 2 * h * f)
        return (-h + 2 * h * f, -h)
    return (-sp * (i + 1), 0)

PATTERNS = {ord("1"): "line", ord("2"): "v", ord("3"): "grid",
            ord("4"): "circle", ord("5"): "square"}

# let sensors warm up
while robot.step(dt) != -1:
    if robot.getTime() > 1.0:
        break
sx, sy, _ = gps.getValues()

if LEADER:
    kb = Keyboard(); kb.enable(dt)
    cd = robot.getSelf().getField("customData")
    pattern, spacing = "line", 8.0
    wps = [(sx + 40, sy), (sx + 40, sy + 40), (sx, sy + 40), (sx, sy)]
    wi, tyaw = 0, 0.0
else:
    leader = robot.getFromDef("DRONE1")
    cd = leader.getField("customData")
    i = ID - 2                              # follower index 0..K-1

# ---------- main loop ----------
while robot.step(dt) != -1:
    t = robot.getTime()
    x, y, z = gps.getValues()
    vx, vy, _ = gps.getSpeedVector()
    roll, pitch, yaw = imu.getRollPitchYaw()
    roll_rate, pitch_rate, _ = gyro.getValues()

    if LEADER:
        key = kb.getKey()
        while key > 0:
            if key in PATTERNS: pattern = PATTERNS[key]
            elif key == ord("]"): spacing += 1
            elif key == ord("["): spacing = max(3, spacing - 1)
            key = kb.getKey()
        cd.setSFString(f"{pattern},{spacing}")

        wx, wy = wps[wi]
        dist = math.hypot(wx - x, wy - y)
        if dist < 2.5:
            wi = (wi + 1) % len(wps)
        if dist > 5:
            tyaw = math.atan2(wy - y, wx - x)
        tx, ty, tz, LIM = wx, wy, ALT, 1.0     # leader flies slowly
    else:
        lx, ly, _ = leader.getPosition()
        R = leader.getOrientation()
        tyaw = math.atan2(R[3], R[0])          # leader heading
        pat, sp = cd.getSFString().split(",")
        f, r = offsets(pat, float(sp), i, K, t)
        tx = lx + f * math.cos(tyaw) + r * math.sin(tyaw)
        ty = ly + f * math.sin(tyaw) - r * math.cos(tyaw)
        tz = ALT + 1.5 * (i % 4)               # altitude layers
        LIM = 2.0

    # position error -> body frame (forward / left)
    ex, ey = tx - x, ty - y
    c, s = math.cos(yaw), math.sin(yaw)
    ef, el = c * ex + s * ey, -s * ex + c * ey
    vf, vl = c * vx + s * vy, -s * vx + c * vy

    pitch_dist = -clamp(0.8 * ef - 1.5 * vf, -LIM, LIM)        # negative = forward
    roll_dist = clamp(0.8 * el - 1.5 * vl, -LIM / 2, LIM / 2)  # positive = left
    yaw_dist = clamp(2.0 * wrap(tyaw - yaw), -1.3, 1.3)
    vertical = 3.0 * clamp(tz - z + 0.6, -1.0, 1.0) ** 3

    roll_in = 50.0 * clamp(roll, -1, 1) + roll_rate + roll_dist
    pitch_in = 30.0 * clamp(pitch, -1, 1) + pitch_rate + pitch_dist
    base = 68.5 + vertical

    motors[0].setVelocity(base - roll_in + pitch_in - yaw_dist)    # front left
    motors[1].setVelocity(-(base + roll_in + pitch_in + yaw_dist)) # front right
    motors[2].setVelocity(-(base - roll_in - pitch_in + yaw_dist)) # rear left
    motors[3].setVelocity(base + roll_in - pitch_in - yaw_dist)    # rear right