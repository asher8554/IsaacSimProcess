# RAPID 출력 신호에 따른 박스 대기·파지·해제와 초기 배치를 검증한다.
import ast
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import line_common  # noqa: E402


class BoxSignalTests(unittest.TestCase):
    def setUp(self):
        # 데모 전체를 import하면 Isaac Sim이 뜨므로 박스 계산 함수와 상태만 실제 소스에서 읽는다.
        # 라인 배치 상수는 두 노드가 같이 쓰는 line_common.py에서 가져온다.
        source = ROOT / "factory_line_demo.py"
        nodes = []
        for node in ast.parse(source.read_text(encoding="utf-8")).body:
            if isinstance(node, ast.FunctionDef) and node.name in {"waiting_box", "box_positions_rapid"}:
                nodes.append(node)
            elif isinstance(node, ast.Assign) and any(isinstance(n, ast.Name) and n.id in {"io", "cell"} for target in node.targets for n in ast.walk(target)):
                nodes.append(node)
        self.ns = {"np": np, **{k: v for k, v in vars(line_common).items() if k.isupper()}}
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec"), self.ns)
        self.positions = self.ns["box_positions_rapid"]

    def test_waits_at_pick_without_grip_even_after_time_passes(self):
        for t in (0.0, 5.0, 12.0):
            with self.subTest(t=t):
                boxes = self.positions(t, np.array([9.0, 9.0, 9.0]))
                np.testing.assert_allclose(boxes[0], [1.7, 0.0, 0.945])
                self.assertIsNone(self.ns["cell"]["held"])
                self.assertEqual(self.ns["cell"]["log"], [])

    def test_grip_follows_tcp_and_release_keeps_last_position(self):
        self.ns["io"]["doGrip"] = 1
        tcp = np.array([1.7, 0.0, 1.12])
        self.positions(0.0, tcp)
        self.assertEqual(self.ns["cell"]["held"], 0)
        tcp = np.array([2.8, -1.5, 0.493])
        np.testing.assert_allclose(self.positions(2.0, tcp)[0], tcp - [0, 0, 0.175])
        self.ns["io"]["doGrip"] = 0
        self.positions(3.0, tcp)
        self.assertIsNone(self.ns["cell"]["held"])
        np.testing.assert_allclose(self.positions(4.0, np.zeros(3))[0], [2.8, -1.5, 0.318])
        self.assertEqual(len(self.ns["cell"]["log"]), 1)
        # 다음 순환에서 벨트로 돌아오면 이전 적재 기록을 지운다.
        np.testing.assert_allclose(self.positions(24.0, np.zeros(3))[0], [-3.7, 0.0, 0.945])
        self.assertNotIn(0, self.ns["cell"]["placed"])

    def test_initial_pallet_boxes_and_belt_keep_their_positions(self):
        boxes = self.positions(-2.0, np.array([9.0, 9.0, 9.0]))
        np.testing.assert_allclose(boxes[:3], [[1.25, 0, 0.945], [-0.55, 0, 0.945], [-2.35, 0, 0.945]])
        np.testing.assert_allclose(boxes[3:], [[2.8, -1.5, 0.318], [3.2, -1.5, 0.318], [3.6, -1.5, 0.318]])


if __name__ == "__main__":
    unittest.main()
