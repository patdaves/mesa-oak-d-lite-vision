import math

from mesa_vision.core import (
    AskManager, Tracker, bearing_from_xz, decide, distance_from_xyz, is_confirmed, lidar_range_at, request_text,
)


def test_bearing_sign_left_positive():
    assert bearing_from_xz(0.0, 2.0) == 0.0
    assert bearing_from_xz(-1.0, 1.0) > 0          # object to the left of the optical axis
    assert abs(bearing_from_xz(1.0, 1.0) + math.pi / 4) < 1e-9


def test_distance():
    assert abs(distance_from_xyz(3.0, 4.0, 12.0) - 13.0) < 1e-9


def _scan(n=360, value=5.0):
    return [value] * n, -math.pi, 2 * math.pi / n


def test_lidar_range_picks_min_in_window_and_skips_invalid():
    r, a0, inc = _scan()
    i = int((0.0 - a0) / inc)
    r[i] = 1.2
    r[i + 1] = float("inf")
    r[i - 1] = float("nan")
    assert abs(lidar_range_at(r, a0, inc, 0.0) - 1.2) < 1e-9


def test_lidar_range_none_when_no_valid_beams():
    r, a0, inc = _scan(value=float("inf"))
    assert lidar_range_at(r, a0, inc, 0.3) is None
    assert lidar_range_at([], 0.0, 0.1, 0.0) is None


def test_confirmation_tolerance():
    assert is_confirmed(1.0, 1.4)
    assert not is_confirmed(1.0, 1.6)
    assert not is_confirmed(1.0, None)


def test_decide_chair_blocking_asks_to_move():
    d = decide("chair", 1.0, 0.05, 1.1)
    assert d.confirmed and d.blocking and d.movable and d.action == "ask_to_move"


def test_decide_person_yields_and_never_asked_to_move():
    assert decide("person", 1.0, 0.0, 1.0).action == "yield"


def test_decide_unconfirmed_is_only_monitored():
    d = decide("chair", 1.0, 0.0, None)
    assert not d.confirmed and d.action == "monitor" and d.source == "none"


def test_decide_not_in_corridor_is_monitored():
    assert decide("chair", 1.0, math.radians(60), 1.0).action == "monitor"


def test_decide_fixed_obstacle_replans():
    assert decide("diningtable", 1.2, 0.0, 1.2).action == "replan"


def test_ask_manager_timeout_and_limit():
    m = AskManager(timeout_s=20.0, max_asks=2)
    assert m.should_ask("chair", 0.0)
    assert not m.should_ask("chair", 10.0)
    assert m.should_ask("chair", 25.0)
    assert not m.should_ask("chair", 100.0) and m.exhausted("chair")
    m.clear("chair")
    assert m.should_ask("chair", 101.0)


def test_request_text():
    assert "move the chair" in request_text("chair")


def test_tracker_confirms_after_min_hits_and_keeps_id():
    tk = Tracker(gate_m=0.8, min_hits=2)
    assert tk.update([(0.0, 0.0, 2.0)], 0.0) == [None]            # first sighting: not yet confirmed
    a = tk.update([(0.1, 0.0, 2.0)], 0.1)[0]
    b = tk.update([(0.2, 0.0, 2.0)], 0.2)[0]
    assert a is not None and b is not None and a.id == b.id


def test_tracker_two_people_keep_separate_ids():
    tk = Tracker()
    tk.update([(-1.0, 0.0, 2.0), (1.0, 0.0, 2.0)], 0.0)
    r = tk.update([(-0.9, 0.0, 2.0), (1.1, 0.0, 2.0)], 0.1)
    assert r[0].id != r[1].id
    r2 = tk.update([(1.2, 0.0, 2.0), (-0.8, 0.0, 2.0)], 0.2)       # input order swapped
    assert r2[0].id == r[1].id and r2[1].id == r[0].id


def test_tracker_gate_creates_new_track():
    tk = Tracker(gate_m=0.5)
    tk.update([(0.0, 0.0, 2.0)], 0.0)
    r = tk.update([(2.0, 0.0, 2.0)], 0.1)
    assert r == [None]                                              # a new, unconfirmed track
    assert len(tk.tracks) == 2


def test_tracker_drops_lost_track_and_reissues_id():
    tk = Tracker(max_misses=2, min_hits=1)
    first = tk.update([(0.0, 0.0, 2.0)], 0.0)[0].id
    for k in range(1, 5):
        tk.update([], 0.1 * k)
    assert tk.tracks == []
    assert tk.update([(0.0, 0.0, 2.0)], 1.0)[0].id != first


def test_tracker_speed_estimate():
    tk = Tracker(min_hits=1, smoothing=1.0)
    tk.update([(0.0, 0.0, 3.0)], 0.0)
    tr = tk.update([(0.05, 0.0, 3.0)], 0.1)[0]
    assert abs(tr.speed_mps - 0.5) < 1e-9
