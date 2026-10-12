"""Run with:  python -m unittest discover -s tests -v   (from the project root)"""
import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]
                       / "controllers" / "swarm_drone"))

from swarm_core import (Peer, pack_state, unpack_state, MSG_SIZE, roles,
                        follower_slot, offsets, repulsion)


def peer(t=0.0, x=0.0, y=0.0, z=10.0, vx=0.0, vy=0.0, yaw=0.0, pat=0, sp=8.0):
    return Peer(t, x, y, z, vx, vy, yaw, pat, sp)


class Messages(unittest.TestCase):
    def test_roundtrip(self):
        data = pack_state(3, 1.5, -2.5, 10.0, 0.5, -0.25, 1.0, 2, 8.0)
        self.assertEqual(len(data), MSG_SIZE)
        pid, p = unpack_state(data, 12.0)
        self.assertEqual(pid, 3)
        self.assertEqual((p.t, p.x, p.y, p.z), (12.0, 1.5, -2.5, 10.0))
        self.assertEqual((p.pattern, p.spacing), (2, 8.0))

    def test_rejects_bad_length(self):
        self.assertIsNone(unpack_state(b"\x01\x02", 0.0))

    def test_rejects_nan(self):
        data = pack_state(2, float("nan"), 0, 10, 0, 0, 0, 0, 8.0)
        self.assertIsNone(unpack_state(data, 0.0))

    def test_rejects_bad_pattern(self):
        self.assertIsNone(unpack_state(pack_state(2, 0, 0, 10, 0, 0, 0, 9, 8.0), 0.0))


class Election(unittest.TestCase):
    def test_lowest_id_leads(self):
        peers = {1: peer(t=9.9), 2: peer(t=9.9), 4: peer(t=9.9)}
        self.assertEqual(roles(3, peers, 10.0, 1.0), (1, [2, 3, 4]))

    def test_failover_when_leader_silent(self):
        peers = {1: peer(t=5.0), 2: peer(t=9.9), 4: peer(t=9.9)}
        self.assertEqual(roles(3, peers, 10.0, 1.0), (2, [3, 4]))

    def test_cascade(self):
        peers = {1: peer(t=1.0), 2: peer(t=1.0), 3: peer(t=9.9)}
        self.assertEqual(roles(4, peers, 10.0, 1.0), (3, [4]))

    def test_alone_means_leader(self):
        self.assertEqual(roles(5, {}, 10.0, 1.0), (5, []))

    def test_fixed_leader_survives_silence(self):
        peers = {2: peer(t=9.9)}
        leader, followers = roles(3, peers, 10.0, 1.0, fixed_leader=1)
        self.assertEqual(leader, 1)
        self.assertEqual(followers, [2, 3])

    def test_slots_close_up_after_failover(self):
        peers = {2: peer(t=9.9), 4: peer(t=9.9), 5: peer(t=9.9)}
        leader, followers = roles(3, peers, 10.0, 1.0)      # drone1 is gone
        self.assertEqual(leader, 2)
        self.assertEqual(follower_slot(3, followers), (0, 3))
        self.assertEqual(follower_slot(5, followers), (2, 3))


class Formations(unittest.TestCase):
    def test_line(self):
        self.assertEqual(offsets("line", 8, 0, 4, 0), (-8, 0))
        self.assertEqual(offsets("line", 8, 3, 4, 0), (-32, 0))

    def test_v_alternates_sides(self):
        self.assertEqual(offsets("v", 8, 0, 4, 0), (-8, -8))
        self.assertEqual(offsets("v", 8, 1, 4, 0), (-8, 8))
        self.assertEqual(offsets("v", 8, 2, 4, 0), (-16, -16))

    def test_unknown_falls_back_to_line(self):
        self.assertEqual(offsets("nope", 8, 1, 4, 0), (-16, 0))


class Repulsion(unittest.TestCase):
    def test_far_peer_no_force(self):
        self.assertEqual(repulsion(1, 0, 0, 10, {2: peer(x=20)}, 0.0, 1.0, 6.0), (0.0, 0.0))

    def test_close_peer_pushes_away(self):
        fx, fy = repulsion(1, 0, 0, 10, {2: peer(x=-3)}, 0.0, 1.0, 6.0)
        self.assertGreater(fx, 0)                 # peer is behind (-x): push forward (+x)

    def test_different_altitude_layer_is_gentler(self):
        same = repulsion(1, 0, 0, 10, {2: peer(x=-3, z=10)}, 0.0, 1.0, 6.0)
        layer = repulsion(1, 0, 0, 10, {2: peer(x=-3, z=11.5)}, 0.0, 1.0, 6.0)
        self.assertLess(math.hypot(*layer), math.hypot(*same))

    def test_stale_peer_ignored(self):
        self.assertEqual(repulsion(1, 0, 0, 10, {2: peer(t=0.0, x=-1)}, 5.0, 1.0, 6.0), (0.0, 0.0))

    def test_capped(self):
        peers = {i: peer(x=-0.5 - 0.01 * i) for i in range(2, 8)}
        self.assertLessEqual(math.hypot(*repulsion(1, 0, 0, 10, peers, 0.0, 1.0, 6.0)), 4.0 + 1e-9)

    def test_stacked_drones_split_by_id(self):
        a = repulsion(1, 0, 0, 10, {2: peer()}, 0.0, 1.0, 6.0)
        b = repulsion(2, 0, 0, 10, {1: peer()}, 0.0, 1.0, 6.0)
        self.assertLess(a[0] * b[0], 0)           # opposite x directions

    def test_head_on_swerves_same_handedness(self):
        # A at origin, B 3 m ahead, facing each other: both should get a y push of the
        # same rotational sense (CCW), i.e. opposite world-y signs for a head-on pair.
        a = repulsion(1, 0, 0, 10, {2: peer(x=3)}, 0.0, 1.0, 6.0)
        b = repulsion(2, 3, 0, 10, {1: peer(x=0)}, 0.0, 1.0, 6.0)
        self.assertNotEqual(a[1], 0.0)
        self.assertLess(a[1] * b[1], 0)


if __name__ == "__main__":
    unittest.main()
