"""brain_view -- what ROS tools are shown of the brain (2026-09-27).

The brain lives outside ROS on purpose (tests/test_ros_containment.py), so
rosbag, rviz and Foxglove could see everything the ROS stack did and
nothing of WHY: which mission, which step, what the brain decided, what it
saw. The bridge closes that by POLLING the brain's own `GET
/mission/status` and republishing it as three ROS topics:

    /brain/status        std_msgs/String -- the status JSON, verbatim, so a
                         bag holds the brain's whole record next to /scan,
                         /odom and /tf on one clock
    /diagnostics         diagnostic_msgs/DiagnosticArray -- one "brain:
                         mission" entry; rqt_robot_monitor and Foxglove's
                         Diagnostics panel read it
    /brain/markers       visualization_msgs/MarkerArray in base_footprint --
                         a caption over the robot, and an arrow at the
                         target's bearing, for rviz and Foxglove's 3D panel

The direction matters: the BRIDGE pulls, the brain is unchanged and still
imports nothing of ROS. The wall stays where it was; this is a window in it.

This module is plain Python -- no rclpy -- so the mapping is unit-tested on a
laptop (tests/test_brain_view.py). bridge.py turns these dicts into
messages.
"""

import math

# diagnostic_msgs/DiagnosticStatus levels.
OK, WARN, ERROR, STALE = 0, 1, 2, 3

# Mission outcomes (control/mission_runner.py) and what each means to an
# operator reading a diagnostics panel. `blocked`, `preempted` and
# `max_steps` ended without failing but did not do the job;
# `arrived_unconfirmed` (3.47) reached something the cloud never checked.
_OUTCOME_LEVEL = {
    "idle": OK, "running": OK, "found": OK, "room_reached": OK, "stopped": OK,
    # 3.31: looked everywhere it could reach; a finding, not a fault.
    "searched": OK,
    "blocked": WARN, "preempted": WARN, "max_steps": WARN,
    "arrived_unconfirmed": WARN,
    "failed": ERROR,
}

CAPTION_HEIGHT_M = 0.35
ARROW_DEFAULT_M = 1.0


def diagnostic(status, error=None):
    """One DiagnosticStatus, as a dict. `status` None means unreachable."""
    if status is None:
        return {"name": "brain: mission", "hardware_id": "brain", "level": STALE,
                "message": f"brain unreachable: {error or 'no answer'}", "values": []}
    outcome = status.get("outcome") or "idle"
    level = _OUTCOME_LEVEL.get(outcome, WARN)
    if status.get("running"):
        message = f"running, step {status.get('step')}/{status.get('max_steps')}"
    else:
        # `error` is set only for `failed`; an arrival the cloud never
        # checked (3.47) carries its reason on the arrival readout instead.
        why = status.get("error") or (
            (status.get("arrival") or {}).get("reason")
            if outcome == "arrived_unconfirmed" else None)
        message = outcome + (f": {why}" if why else "")
    tier = status.get("tier") or {}
    stats = tier.get("stats") or {}
    arrival = status.get("arrival") or {}
    values = [
        ("policy", status.get("policy")),
        ("mission", status.get("mission")),
        ("outcome", outcome),
        ("step", status.get("step")),
        ("max_steps", status.get("max_steps")),
        ("last_action", status.get("last_action")),
        ("vision_failures", status.get("vision_failures")),
        ("tick_rate_hz", status.get("tick_rate_hz")),
        ("cloud_calls", stats.get("cloud_calls")),
        ("frames", stats.get("frames")),
        ("arrival", arrival.get("state")),
        ("arrival_range_m", arrival.get("range_m")),
        ("last_reasoning", (status.get("last_reasoning") or "")[:200]),
    ]
    return {"name": "brain: mission", "hardware_id": "brain", "level": level,
            "message": message,
            "values": [(k, "" if v is None else str(v)) for k, v in values]}


def target_bearing(status):
    """(bearing_deg, range_m or None) of the target, or None.

    The arrival readout wins -- its range is the lidar's, at the bearing --
    then the perception tier's bearing alone. Bearings are the project's
    CLOCKWISE-positive degrees, relative to where the camera points.
    """
    arrival = status.get("arrival") or {}
    if arrival.get("bearing_deg") is not None:
        return float(arrival["bearing_deg"]), arrival.get("range_m")
    perception = status.get("perception") or {}
    if perception.get("bearing_deg") is not None:
        return float(perception["bearing_deg"]), None
    return None


def markers(status, pan_rad=0.0):
    """Marker specs (dicts) for base_footprint. Always two ids, so an arrow
    that is no longer meant is DELETED rather than left drawn."""
    arrived = status is not None and (status.get("arrival") or {}).get("state") == "arrived"
    if status is None:
        text = "brain unreachable"
    elif not status.get("running") and (status.get("outcome") or "idle") == "idle":
        text = "brain idle"
    else:
        head = f"{status.get('policy') or '-'} . step {status.get('step')}/{status.get('max_steps')}"
        text = f"{head}\n{status.get('last_action') or '-'} . {status.get('outcome')}"
    out = [{"id": 0, "type": "text", "text": text,
            "position": (0.0, 0.0, CAPTION_HEIGHT_M), "colour": (1.0, 1.0, 1.0, 1.0)}]
    seen = target_bearing(status) if status is not None else None
    if seen is None:
        out.append({"id": 1, "type": "delete"})
        return out
    bearing_deg, range_m = seen
    # Clockwise-positive project bearing -> ROS's counter-clockwise yaw
    # (REP-103), measured from where the camera points. Composing a panned
    # bearing this way is R3's "hand shortcut", within 0.75 deg for a
    # centred target and up to ~4.6 deg off-centre -- fine for a picture,
    # and the reason nothing here is used to steer.
    yaw = pan_rad - math.radians(bearing_deg)
    out.append({"id": 1, "type": "arrow", "yaw_rad": yaw,
                "length_m": float(range_m) if range_m else ARROW_DEFAULT_M,
                "colour": (0.2, 0.9, 0.3, 1.0) if arrived else (1.0, 0.8, 0.1, 1.0)})
    return out
