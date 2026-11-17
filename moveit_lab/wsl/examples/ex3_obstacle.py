# 예제 3. 기둥을 Planning Scene에 등록하고 OMPL이 피해 가는 경로를 찾게 한다 (RAPID에는 없는 기능)
r"""실행 (WSL): python3 ex3_obstacle.py            기둥을 등록하고 왼쪽 → 오른쪽으로 간다
          python3 ex3_obstacle.py --no-scene  기둥을 등록하지 않는다 (MoveIt이 기둥을 모르는 상태)

배우는 것
  - Planning Scene = MoveIt이 알고 있는 세상. Isaac Sim에 기둥이 "있는 것"과 MoveIt이 기둥을 "아는 것"은 별개다.
    이 예제는 lab_common.OBSTACLE 숫자로 둘을 맞춘다. 실제 현장이라면 셀 도면·비전·포인트클라우드로 채운다.
  - 기둥을 알면 OMPL이 위나 옆으로 돌아가는 경로를 스스로 찾는다. 경유점(RAPID의 중간 robtarget)을 적지 않았다.
  - --no-scene이면 MoveIt은 기둥을 모른다. 경로가 기둥을 지나가면 Isaac Sim에서 팔이 기둥에 걸려 멈추고,
    Isaac Sim 액션 서버가 abort를 돌려준다. 출력은 CONTROL_FAILED가 된다. "계획 성공 ≠ 실행 성공"을 보여 준다.
    (OMPL이 우연히 기둥을 피하는 경로를 고르면 성공할 수도 있다. 여러 번 돌려 본다.)
  - RViz의 MotionPlanning > Scene Objects 탭에서 pillar가 보이는지 확인한다.
"""

import sys
from pathlib import Path

from moveit_client import MoveItClient, named_pose, run

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lab_common import P_LEFT, P_RIGHT  # noqa: E402

# --no-scene이면 기둥을 Planning Scene에서 뺀다. 다음 예제를 돌리면 다시 들어간다 (MoveItClient 기본값).
c = MoveItClient("ex3_obstacle", pillar="--no-scene" not in sys.argv)
run(
    [
        ("왼쪽 목표점으로", lambda: c.move_tcp(P_LEFT)),
        ("오른쪽 목표점으로 (사이에 기둥이 있다)", lambda: c.move_tcp(P_RIGHT)),
        ("ready 자세로 돌아온다", lambda: c.move_joints(named_pose("ready"))),
    ]
)
