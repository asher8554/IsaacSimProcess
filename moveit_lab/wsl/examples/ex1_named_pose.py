# 예제 1. 관절값 목표로 움직인다 — SRDF 이름 자세와 직접 정의한 자세 (RAPID의 MoveAbsJ에 해당)
r"""실행 (WSL): python3 ex1_named_pose.py

배우는 것
  - SRDF group_state = 관절값에 이름을 붙여 둔 자세. RAPID에서 CONST jointtarget을 선언해 두는 것과 같다.
    MoveIt 설정 패키지에 들어 있어 RViz Goal State 목록에도 나온다. 여기서는 "ready"를 쓴다.
  - lab_common.SIDE처럼 코드에 관절값을 직접 적어도 된다. MoveIt에는 같은 "관절 목표"다.
  - plan_only=True면 계획만 한다. RViz에 주황색 궤적이 보이고 Isaac Sim의 로봇은 가만히 있다.
  - plan_only=False면 계획 후 실행한다. move_group이 궤적을 Isaac Sim 액션 서버로 보낸다.
  - 관절 목표라도 경로는 OMPL이 기둥을 피해 찾는다. 같은 목표라도 실행할 때마다 궤적 점 개수가 조금씩 다르다.

바꿔 보기
  - 아래 c.velocity를 0.1로 낮추면 같은 경로를 더 천천히 간다 (출력의 궤적 [s]가 늘어난다).
  - SIDE 대신 named_pose("extended")를 넣어 본다. 계획은 SUCCESS인데 실행은 CONTROL_FAILED가 난다.
    4축이 0 rad인데 Isaac Sim Franka의 4축 상한은 약 -0.07 rad라 거기서 멈추기 때문이다.
    MoveIt이 쓰는 URDF의 한계와 실제(물리) 로봇의 한계가 다르면 "계획 성공 ≠ 실행 성공"이 된다.
  - named_pose("transport")를 넣어 본다. 계획 단계에서 FAILURE다. MoveIt 모델에서 그 자세는 링크5·링크7이 겹친다.
    move_group 터미널에 "Unable to sample any valid states for goal tree"가 찍힌다.
"""

import sys
from pathlib import Path

from moveit_client import ARM_JOINTS, MoveItClient, named_pose, run

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lab_common import SIDE  # noqa: E402

side = dict(zip(ARM_JOINTS, SIDE))  # [rad] {관절 이름: 값}
c = MoveItClient("ex1_named_pose")
c.velocity = 0.3  # [배율] joint_limits.yaml 최대 속도의 30%
run(
    [
        ("SIDE 자세를 계획만 한다 (RViz에서 주황 궤적 확인)", lambda: c.move_joints(side, plan_only=True)),
        ("SIDE 자세로 움직인다 (팔을 왼쪽으로 돌린다)", lambda: c.move_joints(side)),
        ("SRDF 이름 자세 ready로 돌아온다", lambda: c.move_joints(named_pose("ready"))),
    ]
)
