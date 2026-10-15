"""wedge_watch -- when the safety layer keeps refusing nav2, say where.

PLAN-ros-alignment.md 3.68. nav2 plans with the bare chassis; the safety
layer (robot/safety.py) drives only with a side margin, so a passage a
little wider than the chassis is one nav2 will plan into and the robot
server will not let it drive. Nothing tells ROS about the refusal (the
ros2_control plugin ignores the reply to its wheel post), so nav2 keeps the
same plan until its goal times out -- 3.67's garage exit, 1,200 refusals a
run.

This watch is fed every wheel post from driver `ros` and decides, outside
ROS, when that has happened: a run of forward commands zeroed by the
safety layer, with no forward command executed in between and no gap
longer than GAP_S, lasting HOLD_S. The robot made no forward progress in
that run by definition; it can only have turned. The robot server then
marks a keepout spot ahead of the robot (`spot_ahead`) for nav2's planner.

Plain Python and clock-injected, so it is tested without a server.
"""

import math

HOLD_S = 3.0          # forward clamps this long, unbroken, is a wedge
GAP_S = 1.0           # a silence longer than this ends a run
SPOT_AHEAD_M = 0.25   # the spot's centre, ahead of the robot's centre
SPOT_R_M = 0.15       # covers the centre's next 0.10-0.40 m, not its own cell
SPOT_TTL_S = 300.0    # a wrong mark blocks a real passage only this long
MAX_SPOTS = 8         # the oldest is dropped first


def forward_clamped(left_cmd: float, right_cmd: float, reason) -> bool:
    """Did the safety layer zero a FORWARD command? `reason` is
    `vet_wheel_velocity`'s; it names a forward clamp in those words."""
    return bool(reason) and "forward clamped" in reason and (left_cmd + right_cmd) > 0


class WedgeWatch:
    def __init__(self, hold_s: float = HOLD_S, gap_s: float = GAP_S):
        self.hold_s, self.gap_s = hold_s, gap_s
        self.run_start = None
        self.last_clamp = None

    def wheel(self, now: float, left_cmd: float, right_cmd: float, reason) -> bool:
        """One wheel post. True exactly when a wedge is declared; the watch
        then re-arms, so a robot still wedged HOLD_S later fires again."""
        if forward_clamped(left_cmd, right_cmd, reason):
            if self.run_start is None or now - self.last_clamp > self.gap_s:
                self.run_start = now
            self.last_clamp = now
            if now - self.run_start >= self.hold_s - 1e-9:   # clock arithmetic
                self.run_start = now
                return True
            return False
        if (left_cmd + right_cmd) > 0:
            self.run_start = None        # a forward move went through: progress
        return False


def spot_ahead(x_m: float, y_m: float, heading_deg: float,
               ahead_m: float = SPOT_AHEAD_M) -> tuple:
    """The point `ahead_m` in front of a pose, in the house frame (x east,
    y south; compass heading, 0 = north = -y, 90 = +x)."""
    h = math.radians(heading_deg)
    return x_m + ahead_m * math.sin(h), y_m - ahead_m * math.cos(h)


class SpotBook:
    """The live keepout spots: each with an expiry, at most MAX_SPOTS."""

    def __init__(self, ttl_s: float = SPOT_TTL_S, max_spots: int = MAX_SPOTS):
        self.ttl_s, self.max_spots = ttl_s, max_spots
        self.spots = []                  # [{"x_m","y_m","r_m","until"}]

    def add(self, now: float, x_m: float, y_m: float, r_m: float = SPOT_R_M) -> list:
        """Add one; return the live set as the bridge takes it."""
        self.spots = [s for s in self.spots if s["until"] > now]
        self.spots.append({"x_m": x_m, "y_m": y_m, "r_m": r_m, "until": now + self.ttl_s})
        self.spots = self.spots[-self.max_spots:]
        return self.live(now)

    def live(self, now: float) -> list:
        return [{"x_m": round(s["x_m"], 3), "y_m": round(s["y_m"], 3), "r_m": s["r_m"],
                 "ttl_s": round(s["until"] - now, 1)}
                for s in self.spots if s["until"] > now]
