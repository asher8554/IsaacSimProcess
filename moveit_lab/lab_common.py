# MoveIt 실습에서 Isaac Sim(Windows)과 MoveIt 예제(WSL)가 같이 쓰는 로봇 이름·자세·장애물 숫자
r"""두 쪽이 같은 숫자를 봐야 하는 값만 모은 파일이다. ROS 2·Isaac Sim을 import하지 않는다.

    isaac_panda_sim.py      Windows에서 C:\isaacsim-6.2\python.bat으로 실행. 로봇 몸체와 물리
    wsl/examples/*.py       WSL Ubuntu에서 python3로 실행. MoveIt에 "어디로 가라"를 요청하는 쪽

WSL은 E: 드라이브를 /mnt/e로 보므로 두 쪽 모두 이 파일 하나를 그대로 읽는다.
RobotStudio로 치면 "셀 도면"이다. 시뮬레이터에 놓은 장애물과 MoveIt이 아는 장애물이 같은 숫자여야 한다.

내 로봇으로 바꿀 때는 ARM_JOINTS(관절 이름)와 CONTROLLER(MoveIt 컨트롤러 이름)를 그 로봇의 MoveIt 설정에 맞춘다.
"""

# ------------------------------------------------------------------ 로봇과 컨트롤러 이름
# 관절 이름은 MoveIt 설정(URDF)과 Isaac Sim 에셋(USD)에서 글자 하나까지 같아야 한다.
# Franka Panda는 두 쪽 모두 panda_joint1~7이라 그대로 맞는다. 다른 로봇이면 이 목록부터 확인한다.
ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
FINGER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]
# MoveIt이 궤적을 보낼 컨트롤러 이름이다. moveit_resources_panda_moveit_config의 moveit_controllers.yaml에 적힌 값이다.
# MoveIt은 "<CONTROLLER>/follow_joint_trajectory" 액션으로 궤적을 보낸다. Isaac Sim 쪽이 이 이름으로 액션 서버를 연다.
CONTROLLER = "panda_arm_controller"
TRAJECTORY_ACTION = f"/{CONTROLLER}/follow_joint_trajectory"

# ------------------------------------------------------------------ 자세
# [rad] 관절 7개. Panda SRDF의 group_state "ready"와 같은 값이다. 시뮬레이터는 이 자세로 시작한다.
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
# [rad] 관절 7개. 1축을 0.6 rad(+y 쪽) 돌리고 4축을 조금 편 자세. RAPID의 CONST jointtarget처럼 직접 정의한 자세다.
# SRDF의 다른 이름 자세는 이 실습에서 쓰기 어렵다. transport는 MoveIt 모델에서 링크5·링크7이 겹치는 자기 충돌 자세이고,
# extended는 4축이 0 rad라 Isaac Sim Franka의 4축 상한(약 -0.07 rad) 밖이다 (2026-11-17 확인).
SIDE = [0.6, -0.5, 0.0, -2.0, 0.0, 1.571, 0.785]
FINGER_OPEN = 0.04  # [m] 손가락 하나가 가운데에서 벌어진 거리. 이 실습에서는 손가락을 움직이지 않는다

# ------------------------------------------------------------------ 공구 (tooldata에 해당)
# MoveIt의 Panda 모델에는 손끝(TCP) 링크가 없다. 손 링크(panda_hand)에서 손가락 끝까지의 거리를 직접 더한다.
# RAPID의 tooldata tframe에서 z만 있는 공구와 같다. 손 링크 +Z가 손가락이 뻗는 방향이다.
HAND_LINK = "panda_hand"
TCP_Z = 0.1034  # [m] panda_hand 원점에서 손가락 끝 사이 점까지의 거리 (손가락 뿌리 0.0584 + 손가락 길이 약 0.045)

# ------------------------------------------------------------------ 장면 (Planning Scene과 Isaac Sim이 같이 쓰는 값)
# 좌표는 로봇 베이스(panda_link0) 기준이다. 시뮬레이터가 로봇을 월드 원점에 놓으므로 월드 좌표와 같다.
# 길이 단위는 모두 [m]이다.
OBSTACLE = {
    "name": "pillar",  # Planning Scene에서 이 이름으로 보인다 (RViz Scene Objects 탭)
    "center": (0.5, 0.0, 0.225),  # [m] (x, y, z) 기둥의 중심. 로봇 정면 0.5 m
    "size": (0.12, 0.12, 0.45),  # [m] (x 폭, y 폭, 높이). 바닥에서 0.45 m까지 서 있다
}
# 기둥 왼쪽·오른쪽의 두 목표점. 둘 사이를 직선으로 가면 기둥을 뚫는다. MoveIt이 피해 가는지 보는 예제 3에서 쓴다.
# [m] TCP 위치 (x, y, z). 공구는 아래를 본다.
P_LEFT = (0.5, 0.3, 0.2)
P_RIGHT = (0.5, -0.3, 0.2)
# 기둥 위로 지나가는 경유점. 기둥 윗면(0.45 m)보다 TCP가 0.1 m 위다. 예제 2·4의 LIN(= MoveL)에서 쓴다.
# 공구가 아래를 보는 자세로는 x 0.45 m에서 z 0.65 m까지만 닿는다 (2026-11-17 IK 시험). 더 높이면 NO_IK_SOLUTION이 난다.
P_ABOVE_LEFT = (0.45, 0.3, 0.55)  # [m]
P_ABOVE_RIGHT = (0.45, -0.3, 0.55)  # [m]
