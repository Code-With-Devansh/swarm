"""Leader navigation stack: scan -> SLAM -> occupancy grid -> A* -> carrot.

Webots-free (numpy only). Only the current leader runs this; followers don't map.
"""
import math

from occupancy import OccupancyGrid
from slam import Slam2D
from planner import plan, carrot


class LeaderNav:
    def __init__(self, size_m=300.0, res=0.25, max_range=40.0, radius=2.0,
                 lookahead=6.0, replan_period=1.0):
        self.grid = OccupancyGrid(size_m, res)
        self.slam = Slam2D(self.grid, max_range=max_range)
        self.radius = radius
        self.lookahead = lookahead
        self.replan_period = replan_period
        self.path = None
        self.goal = None
        self.t_plan = -1e9
        self.plan_failed = False
        self.active = False

    def start(self, x, y, yaw, t):
        """Begin (or restart) mapping from a known pose; a new leader starts with a blank map."""
        self.grid = OccupancyGrid(self.grid.n * self.grid.res, self.grid.res)
        self.slam = Slam2D(self.grid, max_range=self.slam.max_range)
        self.slam.reset(x, y, yaw, t)
        self.path, self.goal, self.t_plan, self.active = None, None, -1e9, True

    @property
    def pose(self):
        return self.slam.pose

    def scan(self, t, hits, miss, yaw_imu, vel_hint=None):
        return self.slam.update(t, hits, miss, yaw_imu=yaw_imu, vel_hint=vel_hint)

    def target(self, t, goal):
        """Point the leader should fly to this step (the goal itself if the way is clear)."""
        x, y, _ = self.slam.pose
        if (self.goal is None or math.hypot(goal[0] - self.goal[0], goal[1] - self.goal[1]) > 0.5
                or t - self.t_plan >= self.replan_period):
            self.goal = goal
            self.t_plan = t
            self.path = plan(self.grid, (x, y), goal, self.radius)
            self.plan_failed = self.path is None
        if self.path is None:
            return goal                          # no plan: head for the goal and let reactive avoidance cope
        return carrot(self.path, (x, y), self.lookahead)
