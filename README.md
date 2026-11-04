# Drone swarm (Webots R2025a)

Five DJI Mavic 2 Pro drones fly in formation. They are **decentralised**: no drone
reads another's state through the simulator, everything travels over radio
(Emitter/Receiver) at 10 Hz. The leader is elected locally, and drones avoid
each other with a potential field.

## Run
1. Open `worlds/mavic_2_pro.wbt` in Webots and press play.
2. Click the 3D view, then use the keys below.

| Key | Action |
|---|---|
| `1` `2` `3` `4` `5` | line / V / grid / circle / square |
| `]` `[` | spacing +1 / -1 m (min 3) |
| `X` | simulate failure of the current leader (a new leader ignores it for 3 s) |

Console output shows leader changes (`[droneN] leader is droneM`) and, once a
second, the minimum separation between flying drones (`[monitor] ...`).

## Layout
```
code/gen_world.py                     generates the drone block, can inject it into the .wbt
controllers/swarm_drone/swarm_drone.py  controller run by every drone
controllers/swarm_drone/swarm_core.py   pure logic: messages, election, formations, repulsion
controllers/swarm_drone/{lidar_utils,occupancy,slam,planner,leader_nav}.py   lidar SLAM + A* (numpy)
controllers/monitor/monitor.py          supervisor that logs minimum separation
tests/test_core.py                      unit tests (no Webots needed)
worlds/mavic_2_pro.wbt                  the world
```

## How it works
- **Messages**: each drone broadcasts `id, x, y, z, vx, vy, yaw, pattern, spacing`
  (30 bytes). Followers dead-reckon the leader's position between packets.
- **Election**: the lowest id heard within the last 1 s leads. If it goes silent
  the next one takes over and resumes the patrol from the nearest waypoint,
  and the remaining followers close up into the formation. Drones spend the first
  2 s climbing in place, listening, so a late-starting drone never wrongly
  declares itself leader. `controllerArgs ["fixed"]` turns failover off.
- **Avoidance**: each drone offsets its own target away from neighbours inside a
  radius (0.75 x spacing, clamped 2..6 m). Vertical gaps count double, so the
  1.5 m altitude layers (by drone id) rarely trigger it. A small tangential term
  makes head-on pairs swerve the same way.

## Obstacle avoidance (stage 1: leader SLAM + A*)

Every drone carries a 360 degree multi-layer `Lidar` (added by `gen_world.py`, `--lidar-layers 0` removes it).
Needs `numpy` in the Python Webots uses; without it the swarm flies exactly as before.

```
lidar -> level_points (tilt compensation with roll/pitch, mask other drones)
      -> leader only:  Slam2D (scan-to-map match) -> OccupancyGrid -> A* (planner.py) -> carrot
      -> everyone:     obstacle_repulsion (fast reactive layer, added to the drone-drone repulsion)
```

| file | role |
|---|---|
| `lidar_utils.py` | range image -> level 2D points; `mask_peers` drops points on other drones |
| `occupancy.py` | log-odds grid, ray tracing, blurred patch for matching |
| `slam.py` | hill-climbing scan matcher around a constant-velocity prior; yaw from the IMU |
| `planner.py` | obstacle inflation, A* (unknown = free), line-of-sight smoothing, pure-pursuit carrot |
| `leader_nav.py` | glues the above together; a new leader starts with a blank map |

Try it on a route that hits the pines (the default patrol is clear of obstacles):

```
python code/gen_world.py --course --alt 4 --inject worlds/mavic_2_pro.wbt
```

`--alt` matters: the lidar only sees what is at the drone's own height, so check how tall the pines are.
The leader prints `SLAM pose ... vs GPS ...` every 5 s (GPS is ground truth for this log only).

**Stage 1 limits (by design, next steps):**
- The leader's SLAM starts from its GPS pose and the flight controller, the broadcasts and the followers still use GPS, so SLAM runs in the loop for planning but does not yet replace GPS. Stage 2 removes GPS and makes followers locate the leader relative to themselves with their own lidar.
- No loop closure yet: x, y will drift on long runs. Yaw is the IMU's.
- Webots specifics to check on the first run: obstacles need a `boundingObject` to be seen, and `beam_angles(top_first, first_left)` may need flipping if the map comes out mirrored.
- A failed-over leader restarts with an empty map.
- Verified with `tests/test_slam.py` on a synthetic ray-cast world only; not run inside Webots yet.

## Change the swarm
```bash
python code/gen_world.py -n 8 --range 60 --inject worlds/mavic_2_pro.wbt
```
Close the world in Webots first. `--range` limits radio range in metres (try small
values to test lost links); `--fixed-leader` disables failover. The injector
replaces everything from `DEF DRONE1` to the end of the file.
The Emitter/Receiver go in the Mavic2Pro's `bodySlot`; if your PROTO version names it
differently, pass `--slot <name>`.

## Tests
```bash
python -m unittest discover -s tests -v
```

## Known limitations
- Lowest-id-wins election has no hysteresis: if a link flaps, leadership can too,
  and a drone that was cut off may briefly think it is leader (split brain).
- No avoidance of static obstacles (trees, windmills, manor); the patrol
  altitude is simply above them.
- The `X` failure is a motor cut-off, the drone falls where it is.
- Formation changes still send followers straight to their new slots; repulsion
  softens crossings but does not plan around them.
