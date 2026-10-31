"""Check the bundled scheduler against the whiteboard mission contract."""
import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from gazebo_aiworker_patrol.core import Mission, Pose2D, Waypoint
from gazebo_aiworker_patrol.q_scheduler import make_policy, select_action, train, transition


class SchedulerTest(unittest.TestCase):
    def setUp(self):
        home = Pose2D(-.14, 1.05, 0.)
        self.mission = Mission(home, tuple([Waypoint('home', home)] + [
            Waypoint('band_' + str(i), Pose2D(-.28, z, 0.))
            for i, z in enumerate((1.0875, 1.05, 1.0125))]), 1, 120., 30., .01, .2)

    def test_seeded_policy_visits_each_band_once_then_returns_home(self):
        first = train(self.mission, seed=20261030, episodes=4000)
        self.assertEqual(first, train(self.mission, seed=20261030, episodes=4000))
        policy = make_policy(self.mission, first, seed=20261030, episodes=4000)
        current, visited, route = 0, 0, []
        for _ in range(4):
            action = select_action(policy, current, visited, self.mission)
            route.append(action)
            current, visited, done = transition(self.mission, current, visited, action)
        self.assertEqual(route[0], 1)
        self.assertEqual(set(route[:3]), {1, 2, 3})
        self.assertEqual(len(set(route[:3])), 3)
        self.assertEqual(route[3], 0)
        self.assertTrue(done)

    def test_revisiting_a_band_is_rejected(self):
        with self.assertRaises(ValueError):
            transition(self.mission, 1, 1, 1)

    def test_serialized_policy_cannot_be_used_with_wrong_mission(self):
        policy = make_policy(self.mission, train(self.mission, episodes=100), seed=20261030, episodes=100)
        restored = json.loads(json.dumps(policy, allow_nan=False))
        wrong = copy.deepcopy(restored)
        wrong['mission_sha256'] = '0' * 64
        with self.assertRaises(ValueError):
            select_action(wrong, 0, 0, self.mission)
        self.assertIn(select_action(restored, 0, 0, self.mission), (1, 2, 3))


if __name__ == '__main__':
    unittest.main()
