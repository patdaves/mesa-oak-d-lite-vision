"""Pure-Python decision logic (no ROS, no camera) so it can be unit-tested anywhere.

Conventions
-----------
* Camera optical frame: x right, y down, z forward (metres).
* Bearing follows REP-103 yaw: positive = to the left, radians.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

PERSON_LABELS = frozenset({"person"})
# Items a visitor can reasonably be asked to move (PASCAL VOC / COCO names).
MOVABLE_LABELS = frozenset({"chair", "bottle", "backpack", "handbag", "suitcase", "pottedplant"})


def bearing_from_xz(x_m: float, z_m: float) -> float:
    """Bearing (rad, +left) of a point given camera-frame x (right) and z (forward)."""
    return -math.atan2(x_m, z_m)


def distance_from_xyz(x_m: float, y_m: float, z_m: float) -> float:
    return math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m)


def lidar_range_at(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    bearing: float,
    half_window: float = math.radians(5.0),
    range_min: float = 0.05,
    range_max: float = 30.0,
) -> Optional[float]:
    """Smallest valid LiDAR range within +/- half_window of ``bearing``; None if no valid beam."""
    if angle_increment == 0 or not ranges:
        return None
    best = None
    n = len(ranges)
    lo = int(math.floor((bearing - half_window - angle_min) / angle_increment))
    hi = int(math.ceil((bearing + half_window - angle_min) / angle_increment))
    for i in range(max(lo, 0), min(hi, n - 1) + 1):
        r = ranges[i]
        if r is None or math.isnan(r) or math.isinf(r):
            continue
        if r < range_min or r > range_max:
            continue
        if best is None or r < best:
            best = r
    return best


def is_confirmed(camera_dist: float, lidar_dist: Optional[float], tolerance_m: float = 0.5) -> bool:
    """A camera detection is confirmed when LiDAR sees something at a similar range."""
    return lidar_dist is not None and abs(camera_dist - lidar_dist) <= tolerance_m


@dataclass
class Decision:
    confirmed: bool
    source: str  # "lidar" | "depth_only" | "none"
    is_person: bool
    movable: bool
    blocking: bool
    action: str  # "monitor" | "replan" | "yield" | "ask_to_move"


def decide(
    label: str,
    distance_m: float,
    bearing_rad: float,
    lidar_dist: Optional[float],
    *,
    tolerance_m: float = 0.5,
    corridor_half_angle: float = math.radians(20.0),
    blocking_distance_m: float = 1.5,
    allow_depth_only: bool = False,
) -> Decision:
    confirmed = is_confirmed(distance_m, lidar_dist, tolerance_m)
    source = "lidar" if confirmed else ("depth_only" if allow_depth_only else "none")
    trusted = confirmed or allow_depth_only
    person = label in PERSON_LABELS
    movable = label in MOVABLE_LABELS
    blocking = abs(bearing_rad) <= corridor_half_angle and distance_m <= blocking_distance_m
    if not trusted or not blocking:
        action = "monitor"
    elif person:
        action = "yield"
    elif movable:
        action = "ask_to_move"
    else:
        action = "replan"
    return Decision(confirmed, source, person, movable, blocking, action)


def request_text(label: str, language: str = "en") -> str:
    name = label.replace("pottedplant", "plant").replace("diningtable", "table")
    if language == "en":
        return f"Excuse me, could you please move the {name}? It is blocking my way."
    raise ValueError(f"unsupported language: {language}")


@dataclass
class AskManager:
    """Limits how often MESA asks for help with the same obstacle (no nagging)."""

    timeout_s: float = 20.0
    max_asks: int = 2
    _state: dict = field(default_factory=dict)  # label -> (count, last_time)

    def should_ask(self, label: str, now: float) -> bool:
        count, last = self._state.get(label, (0, None))
        if count >= self.max_asks:
            return False
        if last is not None and now - last < self.timeout_s:
            return False
        self._state[label] = (count + 1, now)
        return True

    def exhausted(self, label: str) -> bool:
        return self._state.get(label, (0, None))[0] >= self.max_asks

    def clear(self, label: str) -> None:
        self._state.pop(label, None)


@dataclass
class Track:
    id: int
    x: float
    y: float
    z: float
    hits: int = 1
    misses: int = 0
    speed_mps: float = 0.0
    t_last: float = 0.0


class Tracker:
    """Light-weight multi-person tracker (greedy nearest-neighbour in 3-D, camera frame).

    Follows the idea of SORT (Bewley et al., ICIP 2016): frame-by-frame association of detections to tracks,
    track confirmation after ``min_hits`` matches and deletion after ``max_misses`` missed updates.
    A Kalman filter and an appearance model are deliberately left out for this milestone.
    """

    def __init__(self, gate_m: float = 0.8, max_misses: int = 10, min_hits: int = 2, smoothing: float = 0.5):
        self.gate_m, self.max_misses, self.min_hits, self.smoothing = gate_m, max_misses, min_hits, smoothing
        self.tracks: list = []
        self._next_id = 1

    def update(self, points: Sequence[tuple], t: float) -> list:
        """points: (x, y, z) in metres. Returns, per input point, its Track once confirmed, else None."""
        pairs = []
        for tr in self.tracks:
            for di, (x, y, z) in enumerate(points):
                d = distance_from_xyz(x - tr.x, y - tr.y, z - tr.z)
                if d <= self.gate_m:
                    pairs.append((d, tr.id, di))
        pairs.sort()
        by_id = {tr.id: tr for tr in self.tracks}
        used_ids, assigned = set(), {}
        for d, tid, di in pairs:
            if tid in used_ids or di in assigned:
                continue
            used_ids.add(tid)
            assigned[di] = by_id[tid]
        for di, tr in assigned.items():
            x, y, z = points[di]
            dt = t - tr.t_last
            if dt > 0:
                inst = distance_from_xyz(x - tr.x, y - tr.y, z - tr.z) / dt
                tr.speed_mps = self.smoothing * inst + (1 - self.smoothing) * tr.speed_mps
            tr.x, tr.y, tr.z, tr.t_last = x, y, z, t
            tr.hits += 1
            tr.misses = 0
        for tr in self.tracks:
            if tr.id not in used_ids:
                tr.misses += 1
        self.tracks = [tr for tr in self.tracks if tr.misses <= self.max_misses]
        out = []
        for di, (x, y, z) in enumerate(points):
            tr = assigned.get(di)
            if tr is None:
                tr = Track(self._next_id, x, y, z, t_last=t)
                self._next_id += 1
                self.tracks.append(tr)
            out.append(tr if tr.hits >= self.min_hits else None)
        return out
