"""Failure inference and takeover for the two-robot cell. No simulator required."""

import unittest

from kitmesh.cell_agent import Brain
from kitmesh.cell_layout import CARTONS, PARCEL_REST_Z, PARCELS, ROBOT_A, ROBOT_B, STALL_SECONDS, mark_xyz


def table(**moved):
    """All parcels on their marks, except those given as name=(x, y, z)."""
    out = {pid: list(mark_xyz(pid)) for pid in PARCELS}
    for pid, pos in moved.items():
        out[pid] = list(pos)
    return out


IN_CA = (CARTONS["CA"]["center"][0], CARTONS["CA"]["center"][1], PARCEL_REST_Z)
IN_CB = (CARTONS["CB"]["center"][0], CARTONS["CB"]["center"][1], PARCEL_REST_Z)


class BrainTest(unittest.TestCase):
    def test_own_lane_first_in_plan_order(self):
        b = Brain(ROBOT_B, now=0)
        b.observe(table(), now=0)
        self.assertEqual(b.next_pick(), "P6")
        b.record("P6", {"verified": True}, now=10)
        b.observe(table(P6=IN_CB), now=10)
        self.assertEqual(b.next_pick(), "P5")

    def test_no_takeover_while_other_lane_keeps_moving(self):
        b = Brain(ROBOT_B, now=0)
        b.observe(table(), now=0)
        b.observe(table(P1=IN_CA), now=12)
        b.observe(table(P1=IN_CA, P2=IN_CA), now=30)
        b.observe(table(P1=IN_CA, P2=IN_CA), now=30 + STALL_SECONDS - 1)
        self.assertFalse(b.other_stopped)
        for pid in ("P6", "P5", "P4"):
            b.outcome[pid] = "packed"
        self.assertIsNone(b.next_pick(), "a robot that is still moving parcels is never overtaken")

    def test_stall_is_inferred_only_from_an_unchanged_count(self):
        b = Brain(ROBOT_B, now=0)
        b.observe(table(), now=0)
        b.observe(table(P1=IN_CA), now=12)
        b.observe(table(P1=IN_CA), now=12 + STALL_SECONDS - 0.5)
        self.assertFalse(b.other_stopped)
        b.observe(table(P1=IN_CA), now=12 + STALL_SECONDS)
        self.assertTrue(b.other_stopped)
        self.assertEqual(b.events[-1]["kind"], "peer_stopped")
        self.assertIn("2 parcel(s)", b.events[-1]["note"])

    def test_takeover_after_own_lane_and_from_the_far_end(self):
        b = Brain(ROBOT_B, now=0)
        b.observe(table(P1=IN_CA), now=0)
        b.observe(table(P1=IN_CA), now=STALL_SECONDS + 1)
        self.assertTrue(b.other_stopped)
        self.assertEqual(b.next_pick(), "P6", "own lane still comes first")
        for pid in ("P6", "P5", "P4"):
            b.outcome[pid] = "packed"
        self.assertEqual(b.next_pick(), "P3", "takeover starts where Robot A would have finished")
        b.outcome["P3"] = "packed"
        self.assertEqual(b.next_pick(), "P2")

    def test_takeover_skips_a_parcel_next_to_the_stopped_arm(self):
        b = Brain(ROBOT_B, now=0)
        b.observe(table(), now=0)
        b.observe(table(), now=STALL_SECONDS + 1)
        for pid in ("P6", "P5", "P4"):
            b.outcome[pid] = "packed"
        x, y = PARCELS["P3"]["xy"]
        # Arm link 10 cm on the far side of P3: P3 is within clearance, P2 (24 cm) is not.
        self.assertEqual(b.next_pick([(x + 0.10, y, 0.3)]), "P2")
        self.assertIn("P3", b.blocked)
        self.assertEqual(b.events[-1]["kind"], "blocked")
        self.assertEqual(b.next_pick([(0.0, 0.0, 0.3)]), "P3", "taken once the arm has moved clear")
        self.assertNotIn("P3", b.blocked)

    def test_moving_again_pauses_takeover(self):
        b = Brain(ROBOT_B, now=0)
        b.observe(table(), now=0)
        b.observe(table(), now=STALL_SECONDS + 1)
        self.assertTrue(b.other_stopped)
        b.observe(table(P1=IN_CA), now=STALL_SECONDS + 5)
        self.assertFalse(b.other_stopped)
        self.assertEqual(b.events[-1]["kind"], "peer_moving")

    def test_my_own_takeover_picks_do_not_look_like_the_other_robot_moving(self):
        b = Brain(ROBOT_B, now=0)
        b.observe(table(), now=0)
        b.observe(table(), now=STALL_SECONDS + 1)
        for pid in ("P6", "P5", "P4"):
            b.outcome[pid] = "packed"
        self.assertEqual(b.next_pick(), "P3")
        b.active = "P3"
        b.observe(table(P3=(0.0, -0.45, 0.25), P4=IN_CB, P5=IN_CB, P6=IN_CB), now=STALL_SECONDS + 4)
        self.assertTrue(b.other_stopped, "lifting P3 myself must not cancel the takeover")
        b.active = None
        b.record("P3", {"verified": True}, now=STALL_SECONDS + 12)
        b.observe(table(P3=IN_CB, P4=IN_CB, P5=IN_CB, P6=IN_CB), now=STALL_SECONDS + 12)
        self.assertTrue(b.other_stopped)
        self.assertEqual(b.next_pick(), "P2")

    def test_parcel_gone_is_not_a_failure(self):
        b = Brain(ROBOT_A, now=0)
        b.observe(table(), now=0)
        outcome = b.record("P1", {"verified": False, "reason": "parcel-P1 is not at its mark"}, now=3)
        self.assertEqual(outcome, "gone")

    def test_symmetric_for_robot_a(self):
        b = Brain(ROBOT_A, now=0)
        b.observe(table(), now=0)
        b.observe(table(), now=STALL_SECONDS)
        self.assertTrue(b.other_stopped)
        for pid in ("P1", "P2", "P3"):
            b.outcome[pid] = "packed"
        self.assertEqual(b.next_pick(), "P4")

    def test_parcel_blocked_by_the_stopped_arm_keeps_the_job_open(self):
        b = Brain(ROBOT_B, now=0)
        b.observe(table(P1=IN_CB, P2=IN_CB, P4=IN_CB, P5=IN_CB, P6=IN_CB), now=0)
        b.observe(table(P1=IN_CB, P2=IN_CB, P4=IN_CB, P5=IN_CB, P6=IN_CB), now=STALL_SECONDS + 1)
        b.outcome.update({"P4": "packed", "P5": "packed", "P6": "packed"})
        x, y = PARCELS["P3"]["xy"]
        self.assertIsNone(b.next_pick([(x, y, 0.3)]))
        self.assertFalse(b.done(), "a blocked parcel may still clear")
        b.outcome["P3"] = "review"
        self.assertTrue(b.done(), "a parcel that needs a person does not keep the job open")

if __name__ == "__main__":
    unittest.main()
