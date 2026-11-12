# 산업용 로봇 2대·휴머노이드·AMMR이 한 라인에서 움직이는 Isaac Sim 6.2 공장 자동화 모션 예시
r"""실행: C:\isaacsim-6.2\python.bat factory_line_demo.py [--headless] [--seconds 40]

- 산업용 로봇 A (KUKA KR210)       컨베이어 위를 지나가는 박스를 따라가며 검사하는 동작
- 산업용 로봇 B (Kawasaki RS080N)  컨베이어 끝에서 박스를 들어 팔레트에 내려놓는 동작
- 휴머노이드 (Unitree H1)          라인 옆 통로를 순찰 보행 (Isaac Sim 내장 보행 정책)
- AMMR (Ridgeback + Franka)        팔레트와 출하 버퍼 사이를 왕복하며 팔을 뻗는 동작

ROS 2는 쓰지 않는다. 박스는 물리 없는 시각용 큐브이고 그리퍼 위치를 따라다닐 뿐이다.
"""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="창 없이 실행")
parser.add_argument(
    "--seconds", type=float, default=0.0, help="이 시간(시뮬레이션 초)만 돌리고 자체 점검 후 종료. 0이면 창을 닫을 때까지 실행"
)
args, _ = parser.parse_known_args()
simulation_app = SimulationApp({"headless": args.headless})

import math
import os
import time
import traceback

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.timeline
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import Articulation, RigidPrim, XformPrim
from isaacsim.core.rendering_manager import RenderingManager, ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples import RobotPolicyRunner, get_h1_spec
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

# ------------------------------------------------------------------ 라인 배치
# 주석 맨 앞 대괄호가 그 값의 단위다. [m] 미터, [s] 초, [m/s] 초당 미터, [rad] 라디안(180도 = 3.1416 rad).
# 좌표는 월드 기준이다. x·y가 바닥 평면이고 z가 위쪽이다.
TAKT = 8.0  # [s] 택트 타임. 박스 1개가 나오는 간격
BELT_V = 0.225  # [m/s] 컨베이어 위에서 박스가 움직이는 속도
BELT_Z = 0.77  # [m] 바닥에서 컨베이어 롤러 윗면까지 높이 (ConveyorBelt_A05 실측)
BOX = 0.35  # [m] 박스 한 변의 길이 (정육면체)
X_IN, X_OUT = -3.7, 1.7  # [m] 박스가 벨트에 올라오는 x, 집히는 x. 둘 사이 거리 = 3 * TAKT * BELT_V
GAP = 0.02  # [m] 그리퍼 끝과 박스 윗면 사이 틈
RELEASE = 4.6  # [s] 택트가 시작된 뒤 박스를 놓기까지 걸리는 시간
PALLET_TOP = 0.143  # [m] 바닥에서 팔레트 윗면까지 높이 (pallet.usd 실측)
PALLET_OUT = (3.2, -1.5)  # [m] (x, y) 로봇 B가 박스를 쌓는 팔레트의 중심
PALLET_BUF = (-3.0, -1.5)  # [m] (x, y) AMMR이 다녀오는 출하 버퍼 팔레트의 중심
SLOT_X = (-0.4, 0.0, 0.4)  # [m] 팔레트 중심에서 박스 3칸 각각까지의 x 거리
START_T = -2.0  # [s] 시작 시각. 첫 박스가 집히기 2초 전 상태에서 시작한다
DOWN = np.array([0.0, 0.0, -1.0])  # [단위 없음] (x, y, z) 아래를 가리키는 길이 1짜리 방향 벡터

KUKA_HOME = [0.0, 0.0, 0.3, 0.0, 1.27, 0.0]  # [rad] 관절 a1~a6의 시작 자세. 공구가 아래를 본다
KAWASAKI_HOME = [0.0, 0.2, -1.5, 0.0, -1.44, 0.0] + [0.0] * 6  # [rad] 관절 j1~j6의 시작 자세 + 그리퍼 관절 6개
TUCK = [0.0, -1.0, 0.0, -2.4, 0.0, 1.6, 0.785]  # [rad] AMMR 팔 7축. 접은 자세
REACH = [0.0, 0.75, 0.0, -1.3, 0.0, 2.05, 0.785]  # [rad] AMMR 팔 7축. 앞으로 뻗은 자세
H = math.pi / 2  # [rad] 90도. 방향을 적을 때 쓰는 줄임말
D = PALLET_BUF[0] - PALLET_OUT[0]  # [m] AMMR 출발점에서 버퍼 앞 정차 위치까지의 x 거리 (-6.2)
# 한 줄이 키프레임 하나다. ([s] 시각, [[m] 베이스 x, [m] 베이스 y, [rad] 베이스 방향, [rad] 팔 7축]).
# 마지막 줄의 시각(40)이 반복 주기다.
AMMR_KEYS = [
    (0.0, [0, 0, H, *TUCK]),
    (2.0, [0, 0, H, *REACH]),
    (3.0, [0, 0, H, *REACH]),
    (5.0, [0, 0, H, *TUCK]),
    (7.0, [0, 0, 2 * H, *TUCK]),
    (18.0, [D, 0, 2 * H, *TUCK]),
    (20.0, [D, 0, H, *TUCK]),
    (22.0, [D, 0, H, *REACH]),
    (23.0, [D, 0, H, *REACH]),
    (25.0, [D, 0, H, *TUCK]),
    (27.0, [D, 0, 0, *TUCK]),
    (38.0, [0, 0, 0, *TUCK]),
    (40.0, [0, 0, H, *TUCK]),
]
H1_WAYPOINTS = [(4.0, 4.4), (4.0, 6.0), (-4.0, 6.0), (-4.0, 4.4)]  # [m] (x, y) 순찰 지점 4곳. 반시계 순서로 돈다


# ------------------------------------------------------------------ 모션 계산
def path(keys, t, smooth=True):
    """keys = [(시각, 값), ...]를 주기적으로 보간한다. 마지막 시각이 주기다."""
    times = [k[0] for k in keys]
    t = t % times[-1]
    i = min(int(np.searchsorted(times, t, side="right")) - 1, len(keys) - 2)
    s = (t - times[i]) / (times[i + 1] - times[i])
    if smooth:
        s = s * s * (3.0 - 2.0 * s)
    return (1.0 - s) * np.asarray(keys[i][1], dtype=float) + s * np.asarray(keys[i + 1][1], dtype=float)


def scan_target(t):
    """로봇 A 손끝 목표. 지나가는 박스 위 10 cm를 벨트 속도로 따라간 뒤 되돌아온다."""
    a, b = -1.45, -0.55
    lo, hi = BELT_Z + BOX + 0.10, BELT_Z + BOX + 0.50
    keys = [(0.0, [a, 0, hi]), (1.2, [a, 0, hi]), (2.0, [a, 0, lo]), (6.0, [b, 0, lo]), (6.8, [b, 0, hi]), (TAKT, [a, 0, hi])]
    return path(keys, t, smooth=False)


def palletize_target(t):
    """로봇 B 손끝 목표. 벨트 끝에서 집어 이번 택트의 팔레트 칸에 놓는다."""
    pick = np.array([X_OUT, 0.0, BELT_Z + BOX + GAP])
    place = np.array([PALLET_OUT[0] + SLOT_X[int(t // TAKT) % 3], PALLET_OUT[1], PALLET_TOP + BOX + GAP])
    pick_up, place_up = pick + [0, 0, 0.45], place + [0, 0, 0.60]
    keys = [(0.0, pick), (0.4, pick), (1.4, pick_up), (3.4, place_up), (4.4, place), (RELEASE + 0.2, place)]
    keys += [(5.6, place_up), (7.3, pick_up), (TAKT, pick)]
    return path(keys, t)


def box_positions(t, gripper_pos):
    """박스 6개의 위치. 박스 b는 6 * TAKT 주기로 벨트 → 그리퍼 → 팔레트를 돈다."""
    out = np.zeros((6, 3))
    for b in range(6):
        u = (t - b * TAKT) % (6 * TAKT)  # 0 = 집히는 순간
        if u < RELEASE:  # 그리퍼에 붙어 이동
            out[b] = gripper_pos - [0, 0, GAP + BOX / 2]
        elif u < 3 * TAKT:  # 팔레트 위에서 대기
            slot = SLOT_X[round((t - u) / TAKT) % 3]
            out[b] = [PALLET_OUT[0] + slot, PALLET_OUT[1], PALLET_TOP + BOX / 2]
        else:  # 벨트 위 이동
            out[b] = [X_IN + BELT_V * (u - 3 * TAKT), 0.0, BELT_Z + BOX / 2]
    return out


def ik_step(arm, target, t):
    """손끝을 target으로 보내고 공구 축을 아래로 맞추는 감쇠 최소제곱 미분 IK 한 스텝."""
    pos, quat = arm["ee"].get_world_poses()
    pos, (w, x, y, z) = pos.numpy()[0], quat.numpy()[0]
    rot = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    axis = rot @ arm["tool_axis"]
    error = np.concatenate([target - pos, np.cross(axis, DOWN)])
    jac = arm["robot"].get_jacobian_matrices().numpy()[0, arm["row"], :, :6]
    dq = jac.T @ np.linalg.solve(jac @ jac.T + 0.01 * np.eye(6), error)
    q = arm["robot"].get_dof_positions().numpy()[0, :6]
    # 관절 지령을 스텝당 0.01 rad (= 2 rad/s) 이내로 적분한다. 실제 관절보다 0.1 rad 넘게 앞서가지 않게 묶는다.
    arm["q_cmd"] = np.clip(arm["q_cmd"] + np.clip(0.3 * dq, -0.01, 0.01), q - 0.1, q + 0.1)
    arm["robot"].set_dof_position_targets(arm["q_cmd"], dof_indices=list(range(6)))
    if t > 3.0:  # 자체 점검용 기록 (초기 수렴 구간 제외)
        arm["errors"].append(np.linalg.norm(target - pos))
        arm["aligns"].append(float(axis @ DOWN))


def walk_command():
    """H1을 다음 순찰 지점으로 보내는 [전진 속도, 횡 속도, 회전 속도] 명령."""
    pos, quat = h1.articulation.get_world_poses()
    (x, y, _), (qw, qx, qy, qz) = pos.numpy()[0], quat.numpy()[0]
    gx, gy = H1_WAYPOINTS[state["waypoint"]]
    if math.hypot(gx - x, gy - y) < 0.5:
        state["waypoint"] = (state["waypoint"] + 1) % len(H1_WAYPOINTS)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    turn = (math.atan2(gy - y, gx - x) - yaw + math.pi) % (2 * math.pi) - math.pi
    return np.array([0.5 if abs(turn) < 0.6 else 0.0, 0.0, np.clip(1.5 * turn, -0.7, 0.7)], dtype=np.float32)


# ------------------------------------------------------------------ 장면 구성
# 스테이지(Stage)는 지금 열려 있는 장면 하나다. 그 안의 모든 것(로봇·소품·조명)은 "/World/Kuka" 같은 경로를
# 가진 프림(prim)이다. GUI 오른쪽 Stage 창에 보이는 트리가 이것이다.
# 아래는 빈 장면을 만들고 물리를 쓸 준비를 하는 부분이다. 내 3D 모델로 장면을 짤 때도 이 부분은 그대로 쓴다.

# NVIDIA가 기본으로 제공하는 에셋(로봇·창고·소품)이 있는 서버 주소를 받는다. 이 예시의 모델은 전부 여기서 온다.
# 내 PC에 있는 모델 파일만 쓴다면 이 주소는 필요 없다. 파일 경로를 직접 넘긴다 (아래 add() 설명 참고).
assets = get_assets_root_path()
if assets is None:
    raise RuntimeError("Isaac Sim 에셋 서버에 연결할 수 없다. 인터넷 연결을 확인한다.")

# 아무것도 없는 새 장면을 만든다. 이 장면의 길이 단위는 미터이고 위쪽은 +Z다.
stage_utils.create_new_stage()
# 물리 설정(중력 등)을 담는 프림을 만든다. 설치본 예제와 같은 방식이다.
stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
# 물리 계산을 CPU로 한다. 로봇 몇 대 규모에는 충분하고, 그래픽카드 메모리(8 GB)는 화면 그리기에 남겨 둔다.
SimulationManager.set_physics_sim_device("cpu")
# 앱을 한 프레임 돌려 위 설정이 반영되게 한다.
simulation_app.update()


def add(url, prim_path, position=(0.0, 0.0, 0.0), yaw=0.0):
    """모델 파일(USD) 하나를 장면에 불러와 지정한 위치·방향에 놓는다.

    url        NVIDIA 에셋 서버 안의 파일 경로. 앞에 서버 주소(assets)를 붙여서 연다.
    prim_path  장면 안에서 이 모델에 붙이는 이름(경로). 서로 겹치면 안 된다.
               나중에 Articulation("/World/Kuka")처럼 이 이름으로 로봇을 찾는다.
    position   [m] (x, y, z) 모델의 원점을 놓을 월드 좌표.
    yaw        [rad] 위에서 내려다봤을 때 반시계 방향으로 돌리는 각도.

    내 모델 파일을 놓을 때는 아래 첫 줄의 `assets + url` 자리에 "E:/models/my_robot.usd" 같은
    전체 경로를 넣는다. 그 파일이 다른 파일(메시·재질·payloads 폴더)을 참조한다면 폴더째 같이 둔다.
    """
    # 참조(Reference)로 불러온다. 파일 내용을 장면에 복사하지 않고 "이 경로에 저 파일을 끼운다"고만 적는다.
    # 그래서 같은 파일을 여러 번 놓아도 가볍고(컨베이어 3개, 팔레트 2개) 원본 파일은 바뀌지 않는다.
    # 불러오는 파일의 길이 단위가 장면과 다르면(예: mm로 그린 CAD) 이 함수가 단위를 맞추려고 시도한다.
    # 그래도 놓은 뒤에 크기를 직접 재서 확인한다.
    stage_utils.add_reference_to_stage(assets + url, prim_path)
    # 위치와 방향을 정한다. 방향은 쿼터니언 (w, x, y, z)이고, 아래 값은 z축 둘레로 yaw만큼 돌린 것이다.
    XformPrim(
        prim_path,
        positions=list(position),
        orientations=[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)],
        reset_xform_op_properties=True,
    )


# --- 내 3D 모델 파일로 장면을 짤 때. 파일 형식에 따라 준비가 다르다.
#   .usd / .usda / .usdz    그대로 add()로 놓는다.
#   STEP 같은 CAD            먼저 USD로 바꾼다. Window > Extensions에서 CAD Converter(omni.kit.converter.cad)를
#                            켜고 File > Import 한다. ABB GoFa 실습에서 쓴 방법이다.
#   OBJ / FBX / glTF 메시     먼저 USD로 바꾼다. 설치본 예제 standalone_examples/api/omni.kit.asset_converter 참고.
#   URDF / MJCF              로봇 기술 파일이다. 가져오기 도구로 USD를 만든다.
#                            설치본 예제 standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py 참고.
#   (위 변환 도구들은 설치본에 있는 것만 확인했다. 이 저장소에서 돌려 보지는 않았다.)
#
# 움직이지 않는 소품(설비·작업대·펜스)은 모양만 있어도 놓을 수 있다. 다만 충돌체(Collider)가 없으면 로봇이
# 그대로 통과한다. 아래 컨베이어도 롤러에만 충돌체가 있다.
# 로봇은 모양만으로는 움직이지 못한다. 링크마다 강체(Rigid Body), 링크 사이 관절(Joint), 관절 구동기(Drive),
# Articulation Root가 있어야 Articulation("...")으로 잡아 관절 목표를 보낼 수 있다. GoFa 실습에서 손으로
# 하나씩 만든 것이 이 네 가지다. 그렇게 만든 로봇 USD는 아래 로봇 3대와 같은 방법으로 놓으면 된다.
#
# 놓은 뒤에는 크기와 높이를 실측한다. 위쪽 상수의 BELT_Z, PALLET_TOP도 그렇게 얻은 값이다.

# 환경. 창고 건물·바닥·조명이 파일 하나에 들어 있다. 위치를 안 주면 원점 (0, 0, 0)에 놓인다.
# 바닥에 충돌체가 있어서 H1이 그 위를 걷고 팔레트가 그 위에 놓인다.
add("/Isaac/Environments/Simple_Warehouse/warehouse.usd", "/World/Warehouse")
# 컨베이어. 길이 2 m짜리 같은 파일을 x = -4, -2, 0에 차례로 놓아 x = -4 ~ 2 구간(6 m)을 만든다.
# 이름은 Conveyor_0, Conveyor_1, Conveyor_2로 다르게 준다.
for i in range(3):
    add("/Isaac/Props/Conveyors/ConveyorBelt_A05.usd", f"/World/Conveyor_{i}", (-4.0 + 2.0 * i, 0.0, 0.0))
# 팔레트 2개. 같은 파일을 서로 다른 이름과 위치로 놓는다. z = 0은 바닥이다.
add("/Isaac/Props/Pallet/pallet.usd", "/World/PalletOut", (*PALLET_OUT, 0.0))
add("/Isaac/Props/Pallet/pallet.usd", "/World/PalletBuffer", (*PALLET_BUF, 0.0))
# 흐르는 박스 6개. 파일에서 불러온 것이 아니라 코드로 만든 정육면체 도형이다.
# 경로를 6개 주면 boxes 하나로 6개를 한꺼번에 다룬다. sizes는 [m] 한 변 길이, colors는 (빨강, 초록, 파랑) 0~1 값이다.
# 강체도 충돌체도 없어서 떨어지지 않고 부딪히지도 않는다. 위치는 매 프레임 코드가 직접 정한다.
boxes = Cube([f"/World/Boxes/box_{i}" for i in range(6)], sizes=BOX, colors=[0.72, 0.52, 0.30])
# 버퍼 팔레트 위에 미리 놓아 둔 박스 2개. 움직이지 않는 장식이라 변수에 담지 않는다.
# positions는 [m] 박스마다의 중심 좌표 (x, y, z)다.
Cube(
    [f"/World/BufferBoxes/box_{i}" for i in range(2)],
    sizes=BOX,
    colors=[0.72, 0.52, 0.30],
    positions=[[PALLET_BUF[0] + dx, PALLET_BUF[1], PALLET_TOP + BOX / 2] for dx in (-0.4, 0.0)],
)

# 로봇 3대. (x, y, z)는 [m] 로봇 베이스 원점을 놓는 위치이고 yaw는 [rad] 로봇이 바라보는 방향이다.
# 위치는 손끝이 닿아야 하는 곳까지의 거리를 보고 정한다. 너무 가까우면 팔이 접히고 너무 멀면 닿지 않는다.
# KUKA. 컨베이어 옆 +y 쪽 2.2 m에 놓고 -90도 돌려 컨베이어를 보게 한다.
add("/Isaac/Robots/Kuka/KR210_L150/kr210_l150.usd", "/World/Kuka", (-1.0, 2.2, 0.0), yaw=-H)
# Kawasaki. 컨베이어 끝 너머에 놓고 -135도 돌려 집는 곳과 팔레트의 중간 방향을 보게 한다.
add("/Isaac/Robots/Kawasaki/RS080N/rs080n_onrobot_rg2.usd", "/World/Kawasaki", (3.2, 0.0, 0.0), yaw=-1.5 * H)
# AMMR. 적재 팔레트에서 -y 쪽으로 1.5 m 떨어진 통로에 놓는다. 이 위치가 AMMR 주행 좌표의 원점 (0, 0)이 된다.
add("/Isaac/Robots/Clearpath/RidgebackFranka/ridgeback_franka.usd", "/World/AMMR", (PALLET_OUT[0], -3.0, 0.0))


def make_arm(prim_path, ee_link, tool_axis, home):
    """IK로 움직일 팔 하나를 준비해 필요한 것들을 한 묶음(dict)으로 돌려준다.

    prim_path  add()에서 붙인 로봇 경로.
    ee_link    손끝으로 쓸 링크 이름. robot.link_names를 출력해서 고른다.
    tool_axis  [단위 없음] (x, y, z) 손끝 링크 좌표계에서 공구가 뻗어 나가는 방향.
    home       [rad] 시작 자세. 로봇의 전체 관절 수만큼 적는다.
    """
    # Articulation은 관절로 이어진 로봇 한 대를 다루는 손잡이다. 관절 각도를 읽고 목표를 보낼 때 쓴다.
    robot = Articulation(prim_path)
    robot.set_default_state(dof_positions=home)
    return {
        "robot": robot,
        "ee": RigidPrim(f"{prim_path}/{ee_link}"),
        "row": robot.get_link_indices(ee_link).list()[0] - 1,  # 고정 베이스라 자코비안에 루트 링크가 없다
        "tool_axis": np.array(tool_axis, dtype=float),  # 손끝 링크 좌표계에서 공구가 향하는 축
        "home": np.array(home[:6], dtype=float),
        "q_cmd": np.array(home[:6], dtype=float),  # 적분 중인 관절 지령
        "errors": [],
        "aligns": [],
    }


# ------------------------------------------------------------------ 로봇을 제어 대상으로 잡기
# add()는 모델을 장면에 놓기만 한다. 관절을 읽고 움직이려면 Articulation(경로)으로 잡아야 한다.
# 내 로봇을 넣을 때 정해야 하는 것은 아래 네 가지다.
#   경로        add()에서 붙인 이름.
#   손끝 링크    robot.link_names를 출력해 공구가 달린 링크 이름을 고른다.
#   공구 축      그 링크의 로컬 x·y·z 중 공구가 뻗어 나가는 방향. 관절을 조금씩 움직여 보고 정한다.
#   시작 자세    [rad] 관절 한계 안에서 팔꿈치가 굽은 자세로 정한다. 팔이 곧게 펴진 자세는 IK가 불안정하다.
kuka = make_arm("/World/Kuka", "tool0", [1, 0, 0], KUKA_HOME)  # 손끝은 tool0 링크, 공구 축은 로컬 +X
kawasaki = make_arm("/World/Kawasaki", "gripper_center", [0, 0, 1], KAWASAKI_HOME)  # 그리퍼 중심 링크, 로컬 +Z
# AMMR은 IK 없이 관절 키프레임으로만 움직이므로 Articulation만 만든다.
ammr = Articulation("/World/AMMR")
# 재생을 시작할 때 되돌아갈 기본 자세를 정해 둔다. 값의 개수는 로봇의 전체 관절 수(12)와 같아야 한다.
# 순서는 [m] 베이스 x, [m] 베이스 y, [rad] 베이스 방향, [rad] 팔 7축, [m] 손가락 2개의 벌림이다.
ammr.set_default_state(dof_positions=[0.0, 0.0, 0.0, *TUCK, 0.04, 0.04])

# H1은 add() 대신 정책 러너가 직접 불러온다. 러너가 로봇 모델, 학습된 신경망, 관절 게인을 한 묶음으로 관리한다.
# position은 [m] (x, y, z)이고 z = 1.05는 서 있을 때의 골반 높이다. spawn()이 모델을 장면에 놓는다.
h1 = RobotPolicyRunner(get_h1_spec(), prim_path="/World/H1", position=[-4.0, 4.4, 1.05])
h1.spawn()
# 물리를 계산하는 시간 간격을 정한다. H1 정책이 학습된 간격(0.005 s, 200 Hz)을 그대로 쓴다.
# 보행 정책이 없는 장면이라면 set_physics_dt(1 / 120)처럼 직접 정한다.
SimulationManager.set_physics_dt(h1.physics_dt)
# 화면을 그리는 시간 간격이다. 물리 8스텝마다 한 번 그린다 (0.04 s, 25 Hz).
RenderingManager.set_dt(8 * h1.physics_dt)

# 재생하기 전에 모델 파일을 다 읽을 때까지 기다린다. 서버에서 받는 파일은 시간이 걸린다.
while stage_utils.is_stage_loading():
    simulation_app.update()
# 시작 화면의 카메라를 정한다. eye는 [m] 카메라가 있는 곳, target은 [m] 카메라가 바라보는 점이다.
ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[8.5, -8.0, 6.0], target=[0.0, 0.5, 0.8])

# ------------------------------------------------------------------ 실행
# 실행 중에 바뀌는 값들. t는 [s] 시뮬레이션 시각, first는 "재생 후 첫 스텝인가", waypoint는 H1이 향하는
# 순찰 지점 번호, error는 물리 콜백에서 난 오류 내용이다.
state = {"t": START_T, "first": True, "waypoint": 0, "error": None}


def on_physics_step(dt, context):
    """물리가 한 스텝(0.005 s) 진행될 때마다 Isaac Sim이 불러 주는 함수. 여기서 네 로봇의 목표를 갱신한다.

    dt는 [s] 방금 진행된 물리 한 스텝의 길이다. context는 Isaac Sim이 넘겨주는 값이고 여기서는 쓰지 않는다.
    로봇이 실제로 움직이는 곳이 이 함수다. 내 로봇을 추가했다면 아래 "매 스텝" 부분에 그 로봇의 목표를
    보내는 줄을 넣고, "첫 스텝" 부분에 시작 자세로 되돌리는 줄을 넣는다.
    """
    if state["error"]:  # 앞에서 오류가 났으면 더 진행하지 않는다
        return
    try:
        # --- 첫 스텝. 재생 직후 한 번만 실행된다 (정지 후 다시 재생했을 때도 여기로 온다).
        #     시계와 로봇 자세를 처음 상태로 맞추고 끝낸다.
        if state["first"]:
            state.update(t=START_T, first=False, waypoint=0)
            # 관절 구동기의 게인을 정한다. 앞이 강성 kp, 뒤가 감쇠 kd다. kp가 클수록 목표각으로 세게 당긴다.
            # KUKA 에셋 기본 게인(5.7e16)에서는 지령 없이 a1이 0.13 rad 흘렀다. 다른 팔과 같은 값으로 맞춘다.
            # 내 로봇이 목표를 못 따라가거나 떨면 이 값을 조정한다.
            kuka["robot"].set_dof_gains(5.73e8, 5.73e6)
            for arm in (kuka, kawasaki):
                arm["q_cmd"] = arm["home"].copy()  # IK가 적분해 가는 지령 관절각도 시작 자세로 되돌린다
            for robot in (kuka["robot"], kawasaki["robot"], ammr):
                robot.reset_to_default_state()  # set_default_state로 정해 둔 자세로 한 번에 옮긴다
            # H1을 기본 자세로 세우고 보행 정책을 초기화한다. 인자는 속도 명령 [전진, 횡, 회전]이고 처음에는 0이다.
            h1.restart_from_default_state(np.zeros(3, dtype=np.float32))
            return
        # --- 매 스텝. 시계를 dt만큼 진행시키고, 그 시각에 맞는 목표를 로봇마다 보낸다.
        state["t"] += dt
        t = state["t"]
        ik_step(kuka, scan_target(t), t)  # KUKA 손끝을 "지나가는 박스 위" 좌표로 보낸다
        ik_step(kawasaki, palletize_target(t), t)  # Kawasaki 손끝을 "집기 → 놓기" 경로 위의 좌표로 보낸다
        # AMMR 관절 10개(베이스 3 + 팔 7)에 이 시각의 키프레임 값을 목표로 보낸다. 손가락 2개는 건드리지 않는다.
        ammr.set_dof_position_targets(path(AMMR_KEYS, t), dof_indices=list(range(10)))
        h1.step(dt, walk_command())  # H1 정책에 속도 명령을 넘긴다. 관절 19개의 목표는 정책이 계산한다
    except Exception:  # noqa: BLE001 - 물리 콜백은 예외를 밖으로 던지면 안 된다
        # 오류 내용을 저장해 두면 아래 메인 루프가 그것을 보고 멈춘다.
        state["error"] = traceback.format_exc()
        carb.log_error(f"factory_line_demo: 물리 콜백 실패\n{state['error']}")


# 위 함수를 "물리 한 스텝이 끝난 직후"마다 불리도록 등록한다. 돌려받은 번호는 끝낼 때 등록을 해제하는 데 쓴다.
# 등록만 해 두면 Isaac Sim이 알아서 불러 준다. 내 장면에서도 이 한 줄은 그대로 쓴다.
callback_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)
# 타임라인은 GUI 왼쪽의 재생(▶)·정지(■) 버튼과 같은 것이다. play()는 재생 버튼을 누르는 것과 같다.
# 재생해야 물리가 돌기 시작한다. 그 전까지는 장면이 멈춰 있다.
timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()  # 한 프레임 돌려 재생이 실제로 시작되게 한다

# --- 아래 4줄은 자체 점검(--seconds)용 준비다. 그냥 볼 때는 쓰이지 않는다.
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")  # 화면 캡처를 저장할 폴더 (이 파일 옆 out/)
shots = [6.0, 16.0, 26.0, 36.0] if args.seconds else []  # [s] 화면을 캡처할 시각들
# 점검용 기록값. 순서대로 [m] H1 골반 최저 높이, [m] H1 이동 거리, H1 직전 위치, [m] AMMR 최대 지연,
# [m] AMMR이 가장 멀리 간 x.
h1_z_min, h1_travel, h1_last, ammr_err, ammr_x_min = math.inf, 0.0, None, 0.0, 0.0
wall_start = time.time()  # [s] 실제 시계의 시작 시각. 시뮬레이션이 실시간의 몇 배로 도는지 계산하는 데 쓴다



def frame():
    """한 프레임을 돌린다. 시뮬레이션이 진행 중이면 현재 시각을, 아니면 None을 돌려준다."""
    # 앱을 한 프레임 진행시킨다. 이 안에서 물리 8스텝(= on_physics_step 8번)과 화면 그리기 1번이 일어난다.
    simulation_app.update()
    if timeline.is_stopped():  # GUI에서 정지하면 다음 재생 때 처음부터 다시 시작한다
        if not state["first"]:
            # 정지하면 PhysX가 H1 관절 게인을 에셋 기본값(kp 5730)으로 되돌린다. 정책 런타임을 닫아 두면
            # 다음 재생 때 다시 만들어지면서 학습 게인(kp 150~200)이 재적용된다. 안 닫으면 H1이 넘어진다.
            h1.close()
            state["first"] = True
        return None
    if state["first"]:  # 재생은 했지만 첫 스텝 초기화가 아직 안 끝났다
        return None
    # 박스 6개를 이 시각의 위치로 옮긴다. 두 번째 인자는 [m] Kawasaki 그리퍼 링크의 현재 위치이고,
    # 집힌 박스가 그 위치를 따라가는 데 쓰인다.
    boxes.set_world_poses(positions=box_positions(state["t"], kawasaki["ee"].get_world_poses()[0].numpy()[0]))
    return state["t"]


# ------------------------------------------------------------------ 메인 루프
# 창이 닫히거나 오류가 날 때까지 프레임을 반복한다. 로봇을 움직이는 일은 위의 물리 콜백이 한다.
# 이 루프가 하는 일은 프레임을 한 장씩 진행시키고(frame) 박스를 옮기는 것뿐이다.
# --seconds 없이 실행했다면 아래 "자체 점검용 기록"은 건너뛰고 창을 닫을 때까지 돈다.
while simulation_app.is_running() and not state["error"]:
    t = frame()
    if t is None or not args.seconds:
        continue
    # --- 아래는 자체 점검용 기록 (--seconds를 줬을 때만 실행된다)
    h1_pos = h1.articulation.get_world_poses()[0].numpy()[0]  # [m] H1 골반의 월드 좌표 (x, y, z)
    h1_z_min = min(h1_z_min, h1_pos[2])  # 서 있으면 0.87 이상이고 넘어지면 0.15 근처까지 떨어진다
    h1_travel += 0.0 if h1_last is None else float(np.linalg.norm(h1_pos[:2] - h1_last))  # 바닥 위 이동 거리 누적
    h1_last = h1_pos[:2].copy()
    ammr_q = ammr.get_dof_positions().numpy()[0]  # AMMR 관절 12개의 현재 값. 0번이 [m] 베이스 x다
    ammr_err = max(ammr_err, abs(ammr_q[0] - path(AMMR_KEYS, t)[0]))  # 베이스 x가 목표에서 가장 멀어진 거리
    ammr_x_min = min(ammr_x_min, ammr_q[0])
    if shots and t >= shots[0]:  # 정해 둔 시각이 되면 화면을 PNG 파일로 저장한다
        os.makedirs(out_dir, exist_ok=True)
        capture_viewport_to_file(get_active_viewport(), os.path.join(out_dir, f"shot_{int(shots.pop(0)):02d}s.png"))
    if t >= args.seconds:  # 정해 둔 시간이 지나면 루프를 끝내고 아래 점검으로 넘어간다
        break

# ------------------------------------------------------------------ 자체 점검 (--seconds를 줬을 때만)
# 화면으로 봐서는 놓치기 쉬운 것을 숫자로 검사한다. 각 줄은 (항목 이름, 통과 여부, 측정값)이다.
# 내 장면으로 바꿨다면 항목도 내 로봇에 맞게 고친다. "목표를 따라가는가"와 "넘어지지 않는가"는 남겨 둔다.

if args.seconds and state["error"]:
    print("SELF-CHECK FAIL | physics callback raised\n" + state["error"])
elif args.seconds:
    for _ in range(5):  # 마지막 캡처가 파일로 써지도록 몇 프레임 더 돈다
        simulation_app.update()
    elapsed = state["t"] - START_T  # [s] 시뮬레이션 안에서 흐른 시간
    # errors는 ik_step이 스텝마다 쌓아 둔 [m] 손끝 목표와 실제 위치의 거리다.
    # aligns는 공구 축과 아래 방향 사이 각도의 코사인이다. 1이면 정확히 아래를 본다.
    checks = [
        ("KUKA mean tip error < 0.03 m", np.mean(kuka["errors"]) < 0.03, f"mean {np.mean(kuka['errors']):.4f} max {np.max(kuka['errors']):.4f}"),
        ("KUKA tool points down", np.mean(kuka["aligns"]) > 0.95, f"mean cos {np.mean(kuka['aligns']):.3f}"),
        ("Kawasaki mean tip error < 0.03 m", np.mean(kawasaki["errors"]) < 0.03, f"mean {np.mean(kawasaki['errors']):.4f} max {np.max(kawasaki['errors']):.4f}"),
        ("Kawasaki tool points down", np.mean(kawasaki["aligns"]) > 0.95, f"mean cos {np.mean(kawasaki['aligns']):.3f}"),
        ("AMMR base follows its path (< 0.3 m)", ammr_err < 0.3, f"max lag {ammr_err:.3f} m, reached x {ammr_x_min:.2f} m"),
        ("H1 stays upright (pelvis > 0.8 m)", h1_z_min > 0.8, f"min z {h1_z_min:.3f} m"),
        ("H1 keeps walking (> 0.25 m/s)", h1_travel / elapsed > 0.25, f"{h1_travel:.1f} m in {elapsed:.0f} s"),
    ]
    realtime = elapsed / (time.time() - wall_start)  # 1이면 실시간, 0.7이면 실제 1초에 시뮬레이션 0.7초
    # --- 정지 후 재생 점검. GUI에서 정지(■)했다가 재생(▶)하는 것을 코드로 똑같이 해 본다.
    #     다시 재생한 뒤 10초 동안 H1 높이와 두 팔의 손끝 오차를 본다.
    seen = len(kuka["errors"]), len(kawasaki["errors"])  # 여기까지 쌓인 기록 개수. 재생 뒤 기록만 골라내는 데 쓴다
    timeline.stop()
    for _ in range(5):  # 정지 상태로 몇 프레임 돌린다. frame()이 정지를 알아채고 H1 정책을 닫는다
        frame()
    timeline.play()
    replay_z = math.inf  # [m] 재생 뒤 H1 골반 최저 높이
    while simulation_app.is_running() and not state["error"]:
        t = frame()
        if t is None:
            continue
        replay_z = min(replay_z, h1.articulation.get_world_poses()[0].numpy()[0][2])
        if t >= 8.0:  # 시계가 -2에서 다시 시작하므로 8이 되면 10초가 지난 것이다
            break
    replay_err = np.mean(kuka["errors"][seen[0] :] + kawasaki["errors"][seen[1] :])  # [m] 재생 뒤 두 팔의 평균 오차
    checks += [
        ("Replay ran without error", state["error"] is None, state["error"] or "ok"),
        ("H1 upright after stop and replay (pelvis > 0.8 m)", replay_z > 0.8, f"min z {replay_z:.3f} m"),
        ("Arms track after stop and replay (< 0.03 m)", replay_err < 0.03, f"mean {replay_err:.4f}"),
    ]
    # 결과를 한 줄씩 출력한다. 하나라도 FAIL이면 마지막 줄이 SELF-CHECK FAIL이 된다.
    for name, ok, detail in checks:
        print(f"CHECK {'PASS' if ok else 'FAIL'} | {name} | {detail}")
    print(f"REALTIME {realtime:.2f}x ({elapsed:.0f} sim s)")
    print("SELF-CHECK", "PASS" if all(ok for _, ok, _ in checks) else "FAIL")

# ------------------------------------------------------------------ 종료
# 창을 닫았거나 자체 점검이 끝나면 여기로 온다. 시작할 때 만든 것을 반대 순서로 정리한다.
timeline.stop()  # 정지 버튼(■)을 누르는 것과 같다. 물리가 멈추고 장면이 재생 전 상태로 돌아간다
SimulationManager.deregister_callback(callback_id)  # on_physics_step이 더는 불리지 않게 등록을 해제한다
h1.close()  # H1 보행 정책이 쓰던 신경망과 구동기 자원을 놓아준다
simulation_app.close()  # Isaac Sim 프로그램 자체를 끝낸다
