"""ROS-independent smooth figure-eight controller."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

@dataclass(frozen=True)
class Command:
    linear_x: float = 0.0
    angular_z: float = 0.0

@dataclass(frozen=True)
class Config:
    amplitude: float = 0.66
    samples: int = 721
    lookahead: float = 0.20
    goal_tolerance: float = 0.07
    cross_track_limit: float = 0.50
    max_linear: float = 0.66
    min_linear: float = 0.21
    max_angular: float = 1.0
    linear_acceleration: float = 0.8
    angular_acceleration: float = 2.0

def make_path(c: Config, origin: Pose):
    """Gerono curve rotated so the robot enters it straight ahead."""
    if c.samples < 101 or c.min_linear > c.max_linear:
        raise ValueError("invalid configuration")
    rotation = origin.yaw - math.pi / 4
    co, si = math.cos(rotation), math.sin(rotation)
    path, travelled = [], 0.0
    for i in range(c.samples):
        t = 2 * math.pi * i / (c.samples - 1)
        rx, ry = c.amplitude * math.sin(t), c.amplitude * math.sin(t) * math.cos(t)
        x, y = origin.x + rx * co - ry * si, origin.y + rx * si + ry * co
        if path:
            travelled += math.hypot(x - path[-1][0], y - path[-1][1])
        path.append((x, y, travelled))
    return path

class Tracker:
    """Pure pursuit with monotonic progress through the crossing."""
    def __init__(self, config=Config()):
        self.c, self.path, self.index = config, [], 0
        self.active, self.completed, self.last = False, False, Command()

    def start(self, pose):
        self.path, self.index = make_path(self.c, pose), 0
        self.active, self.completed, self.last = True, False, Command()

    def stop(self):
        self.active, self.last = False, Command()
        return self.last

    def update(self, pose, dt):
        if not self.active or pose is None or dt <= 0:
            return self.stop()
        end = min(len(self.path), self.index + 81)
        nearest = min(range(self.index, end), key=lambda i: (self.path[i][0]-pose.x)**2 + (self.path[i][1]-pose.y)**2)
        px, py, distance = self.path[nearest]
        if math.hypot(px-pose.x, py-pose.y) > self.c.cross_track_limit:
            return self.stop()
        self.index = nearest
        gx, gy, total = self.path[-1]
        if nearest >= len(self.path)-12 and math.hypot(gx-pose.x, gy-pose.y) <= self.c.goal_tolerance:
            self.completed = True
            return self.stop()
        target = nearest
        while target+1 < len(self.path) and self.path[target][2] < distance+self.c.lookahead:
            target += 1
        dx, dy = self.path[target][0]-pose.x, self.path[target][1]-pose.y
        alpha = math.atan2(math.sin(math.atan2(dy, dx)-pose.yaw), math.cos(math.atan2(dy, dx)-pose.yaw))
        curvature = 2*math.sin(alpha)/max(math.hypot(dx, dy), 1e-6)
        desired_v = self.c.max_linear
        desired_w = max(-self.c.max_angular, min(self.c.max_angular, desired_v*curvature))
        v = self._slew(self.last.linear_x, desired_v, self.c.linear_acceleration*dt)
        w = self._slew(self.last.angular_z, desired_w, self.c.angular_acceleration*dt)
        self.last = Command(v, w)
        return self.last

    @staticmethod
    def _slew(current, desired, step):
        return current + max(-step, min(step, desired-current))
