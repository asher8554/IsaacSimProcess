# RAPID 컨트롤러 노드의 메시지 변환·시계 되감기 처리·지령 모양을 Isaac Sim 없이 검증한다.
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import line_common  # noqa: E402  (rclpy 경로와 RMW를 먼저 정한다)
import rapid_controller as rc  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402


class Sink:
    """publish()로 받은 메시지를 모아 두는 가짜 발행기."""

    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class RapidControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_transform_and_stamp_round_trip(self):
        rot = Rotation.from_euler("ZYX", [30, -10, 170], degrees=True)
        pos, back = line_common.from_transform(line_common.to_transform(12.345, [1.0, -2.0, 0.5], rot, "tGripper"))
        np.testing.assert_allclose(pos, [1.0, -2.0, 0.5])
        np.testing.assert_allclose(back.as_matrix(), rot.as_matrix(), atol=1e-9)
        self.assertAlmostEqual(line_common.seconds(line_common.stamp(12.345)), 12.345, places=6)

    def test_steps_programs_on_clock_and_restarts_when_clock_rewinds(self):
        node = rc.RapidController()
        try:
            node.pub = {name: Sink() for name in node.pub}
            down = Rotation.from_euler("ZYX", [0, 0, 180], degrees=True)
            node.latest["/kuka/flange_pose"] = line_common.to_transform(0.0, [-1.0, 0.5, 1.5], down, "kuka/tool0")
            node.latest["/kawasaki/flange_pose"] = line_common.to_transform(0.0, [2.0, 0.0, 1.5], down, "kawasaki/gripper_center")
            node.latest["/ammr/joint_states"] = JointState(name=[f"j{i}" for i in range(10)], position=[0.0] * 3 + list(rc.TUCK))
            node.latest["/h1/pose"] = PoseStamped()
            for t in (0.005, 0.010, 0.015):
                node.on_clock(Clock(clock=line_common.stamp(t)))
            first_tasks = node.tasks
            # 첫 시각에 태스크를 만들고, 다음 두 시각에 지령을 낸다. 팔 지령에는 tool 이름이 실린다.
            self.assertEqual(len(node.pub["kawasaki"].sent), 2)
            self.assertEqual(node.pub["kawasaki"].sent[-1].child_frame_id, "tGripper")
            self.assertEqual(node.pub["kuka"].sent[-1].child_frame_id, "tScanner")
            # AMMR 지령은 베이스 가상 관절 3개 + 팔 7축이고, 시작 자리(AMMR_HOME)가 관절값 0이다.
            ammr = node.pub["ammr"].sent[-1]
            self.assertEqual(len(ammr.position), 10)
            np.testing.assert_allclose(ammr.position[:2], [0.0, 0.0], atol=1e-3)
            # 시계가 되돌아가면(정지 후 재생) 프로그램을 처음부터 다시 만든다.
            node.on_clock(Clock(clock=line_common.stamp(0.005)))
            self.assertIsNotNone(node.tasks)
            self.assertIsNot(node.tasks, first_tasks)
        finally:
            node.destroy_node()

    def test_unregistered_tool_names_the_problem(self):
        node = rc.RapidController()
        original = rc.tScanner
        try:
            rc.tScanner = line_common.tooldata(0.0, 0.0, 0.0, 0.0, 90.0, 0.0)  # 값은 같지만 TOOLS에 등록되지 않은 공구
            node.pub = {name: Sink() for name in node.pub}
            down = Rotation.from_euler("ZYX", [0, 0, 180], degrees=True)
            node.latest["/kuka/flange_pose"] = line_common.to_transform(0.0, [-1.0, 0.5, 1.5], down, "kuka/tool0")
            node.latest["/kawasaki/flange_pose"] = line_common.to_transform(0.0, [2.0, 0.0, 1.5], down, "kawasaki/gripper_center")
            node.latest["/ammr/joint_states"] = JointState(name=[f"j{i}" for i in range(10)], position=[0.0] * 3 + list(rc.TUCK))
            node.latest["/h1/pose"] = PoseStamped()
            node.on_clock(Clock(clock=line_common.stamp(0.005)))
            with self.assertRaisesRegex(ValueError, "TOOLS"):
                node.on_clock(Clock(clock=line_common.stamp(0.010)))
        finally:
            rc.tScanner = original
            node.destroy_node()

    def test_walk_command_turns_before_walking(self):
        self.assertEqual(rc.walk_command((0.0, 0.0, 0.0), None), (0.0, 0.0, 0.0))
        forward, _, turn = rc.walk_command((0.0, 0.0, 0.0), (5.0, 0.0))
        self.assertEqual((forward, turn), (0.5, 0.0))
        forward, _, turn = rc.walk_command((0.0, 0.0, 0.0), (0.0, 5.0))
        self.assertEqual(forward, 0.0)
        self.assertAlmostEqual(turn, 0.7)


if __name__ == "__main__":
    unittest.main()
