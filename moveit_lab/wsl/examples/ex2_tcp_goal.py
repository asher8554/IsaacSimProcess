# 예제 2. TCP 좌표로 목표를 주고 계획기 세 가지(OMPL·PTP·LIN)를 비교한다 (robtarget + MoveJ·MoveL)
r"""실행 (WSL): python3 ex2_tcp_goal.py

배우는 것
  - TCP 목표 (x, y, z, rx, ry, rz)를 주면 MoveIt이 IK(KDL)로 관절값을 푼다. RAPID의 robtarget과 같다.
    MoveIt에는 tooldata가 없어서 moveit_client.hand_pose()가 TCP를 손 링크 목표로 바꿔 준다.
  - OMPL      경로를 자유롭게 찾는다. 결과 경로가 매번 다르고 곡선일 수 있다.
  - Pilz PTP  관절 공간 직선. 모든 축이 같이 출발·같이 도착한다. RAPID MoveJ와 가장 비슷하다.
  - Pilz LIN  TCP가 공간에서 직선으로 간다. RAPID MoveL과 같다.
  - 출력의 "궤적 점" 수와 길이를 비교한다. 같은 거리라도 LIN은 TCP 직선을 지키느라 관절이 더 많이 움직일 수 있다.

좌표는 lab_common.py에 있다. Isaac Sim 화면의 초록 표시가 P_LEFT·P_RIGHT다. 이 예제는 기둥 위쪽만 쓴다.
"""

import sys
from pathlib import Path

from moveit_client import MoveItClient, named_pose, run

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lab_common import P_ABOVE_LEFT, P_ABOVE_RIGHT, P_LEFT  # noqa: E402

c = MoveItClient("ex2_tcp_goal")
run(
    [
        ("OMPL: 왼쪽 위 점으로 (경로는 MoveIt 마음대로)", lambda: c.move_tcp(P_ABOVE_LEFT)),
        ("Pilz LIN: 왼쪽 목표점으로 곧장 내려간다 (MoveL)", lambda: c.move_tcp(P_LEFT, planner="LIN")),
        ("Pilz LIN: 다시 위로 (MoveL)", lambda: c.move_tcp(P_ABOVE_LEFT, planner="LIN")),
        ("Pilz PTP: 오른쪽 위 점으로 (MoveJ)", lambda: c.move_tcp(P_ABOVE_RIGHT, planner="PTP")),
        ("ready 자세로 돌아온다", lambda: c.move_joints(named_pose("ready"))),
    ]
)
