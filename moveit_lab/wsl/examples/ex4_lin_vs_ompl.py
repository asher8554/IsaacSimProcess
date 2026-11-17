# 예제 4. 직선 이동(LIN = MoveL)은 장애물을 피하지 못한다. 경유점을 사람이 적는 RAPID 방식과 OMPL 방식 비교
r"""실행 (WSL): python3 ex4_lin_vs_ompl.py   (시작할 때 기둥을 Planning Scene에 넣는다)

배우는 것
  - Pilz LIN은 TCP 직선만 만든다. 직선이 기둥을 지나면 MoveIt은 실행하지 않고 실패를 돌려준다.
    여기서는 plan_only=True로 계획만 시켜서 결과 코드만 본다 (실패가 정상이다).
  - RAPID 방식: 사람이 기둥 위 경유점 두 개를 적고 MoveL 세 번으로 넘어간다. 경로가 항상 똑같다.
  - OMPL 방식: 경유점 없이 목표만 준다. 경로는 MoveIt이 찾고 매번 조금씩 다르다.
  - 현장에서 반복 사이클은 RAPID 방식(정해진 경로)을, 위치가 매번 바뀌는 작업은 OMPL을 쓰는 이유가 이것이다.
"""

import sys
from pathlib import Path

from moveit_client import MoveItClient, named_pose, run

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lab_common import P_ABOVE_LEFT, P_ABOVE_RIGHT, P_LEFT, P_RIGHT  # noqa: E402

c = MoveItClient("ex4_lin_vs_ompl")
run(
    [
        # PTP(관절 공간 직선)로 가면 ready에서 왼쪽 목표점으로 가는 길에 손이 기둥을 쓸고 지나가 INVALID_MOTION_PLAN이 난다.
        # 그래서 시작점까지는 OMPL로 간다.
        ("OMPL: 왼쪽 목표점으로", lambda: c.move_tcp(P_LEFT)),
        # 실패해야 정상인 단계. 결과를 뒤집어 "실패했으면 통과"로 센다.
        ("LIN: 기둥을 뚫는 직선 → 계획 실패가 정상", lambda: not c.move_tcp(P_RIGHT, planner="LIN", plan_only=True)),
        ("RAPID 방식 1/3: LIN 위로", lambda: c.move_tcp(P_ABOVE_LEFT, planner="LIN")),
        ("RAPID 방식 2/3: LIN 기둥 위를 건너감", lambda: c.move_tcp(P_ABOVE_RIGHT, planner="LIN")),
        ("RAPID 방식 3/3: LIN 아래로", lambda: c.move_tcp(P_RIGHT, planner="LIN")),
        ("OMPL 방식: 경유점 없이 왼쪽 목표점으로 되돌아간다", lambda: c.move_tcp(P_LEFT)),
        ("ready 자세로 돌아온다", lambda: c.move_joints(named_pose("ready"))),
    ]
)
