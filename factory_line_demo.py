# 산업용 로봇 2대·휴머노이드·AMMR이 한 라인에서 움직이는 Isaac Sim 6.2 공장 자동화 모션 예시
r"""실행: C:\isaacsim-6.2\python.bat factory_line_demo.py [--headless] [--seconds 40] [--no-controller]

- 산업용 로봇 A (KUKA KR210)       컨베이어 위를 지나가는 박스를 따라가며 검사하는 동작
- 산업용 로봇 B (Kawasaki RS080N)  컨베이어 끝에서 박스를 들어 팔레트에 내려놓는 동작
- 휴머노이드 (Unitree H1)          라인 옆 통로를 순찰 보행 (Isaac Sim 내장 보행 정책)
- AMMR (Ridgeback + Franka)        팔레트와 출하 버퍼 사이를 왕복하며 팔을 뻗는 동작

ROS 2 노드 두 개로 나뉘어 있다. 이 파일은 그중 Isaac Sim 노드(/isaac_sim)다.

    /isaac_sim (이 파일)          로봇 본체와 저수준 제어. 관절 구동기, 미분 IK, H1 보행 정책, 박스와 센서
    /rapid_controller             RAPID 스타일 프로그램 4개 (rapid_controller.py). 별도 프로세스다

두 노드는 서로의 코드를 부르지 않는다. topic과 service로만 대화한다 (목록은 rapid_controller.py 맨 위).
이 파일은 RAPID 프로그램을 모른다. 지령 topic이 오면 그대로 로봇에 적용하고, 로봇 상태와 센서 값을 topic으로 내보낸다.
기본 실행은 rapid_controller.py를 자식 프로세스로 같이 띄운다. --no-controller면 띄우지 않고 기다린다.

제어 흐름 (물리 0.005초마다).
    이 파일: 로봇 상태·센서 발행 → /clock 발행
    rapid_controller.py: /clock 수신 → RapidTask.step() → 지령 발행
    이 파일: 지령 수신 → IK·관절 목표·보행 정책 → Isaac Sim 관절 구동기
박스는 물리 없는 시각용 큐브이고 그리퍼 위치를 따라다닐 뿐이다.

실행 중에 물체를 클릭하면 그 물체 윗면 가운데 좌표가 robtarget 한 줄로 출력된다 (on_stage_event 참고).
"""

import argparse
import os
import subprocess
import sys

# ROS 2 통신 구현(RMW)을 FastDDS로 정한다. Isaac Sim의 ROS 2 브리지가 시작할 때 이 값을 읽으므로 SimulationApp보다 먼저 정한다.
# python.bat의 기본값은 Zenoh인데, Zenoh는 라우터를 따로 띄워야 프로세스끼리 찾는다. 컨트롤러도 같은 값을 쓴다 (line_common.RMW).
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

# 이 스크립트의 수치 계산은 6x6 이하의 작은 행렬뿐이다. numpy·scipy·torch가 계산용 스레드를 여러 개 띄우면
# 도움은 안 되고 물리·화면 스레드와 CPU를 다투기만 한다. Isaac Sim을 띄우기 전에 1개로 묶는다.
# 사용자가 환경 변수로 이미 값을 정해 두었으면 그 값을 따른다.
for _name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_name, "1")

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="창 없이 실행")
parser.add_argument(
    "--seconds", type=float, default=0.0, help="이 시간(시뮬레이션 초)만 돌리고 자체 점검 후 종료. 0이면 창을 닫을 때까지 실행"
)
parser.add_argument(
    "--no-controller", action="store_true", help="rapid_controller.py를 띄우지 않는다. 다른 터미널에서 직접 실행할 때 쓴다"
)
args = parser.parse_args()
# 별도 Isaac Sim 프로세스에 명령을 보내는 구조가 아니다. 이 Python 실행 안에서 앱과 물리 엔진을 시작한다.
print("[MOTION] ros2 | /rapid_controller (RAPID programs) -> topics -> /isaac_sim (robot drives)", flush=True)
simulation_app = SimulationApp({"headless": args.headless})

import math
import time
import traceback

import carb
import isaacsim.core.experimental.utils.app as app_utils
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
import omni.usd
from pxr import Usd, UsdGeom
from rapid_style import robtarget
from scipy.spatial.transform import Rotation

# ROS 2 브리지 확장을 켠다. Isaac Sim에 들어 있는 ROS 2 Jazzy 라이브러리를 불러오는 단계다.
# 이 줄 뒤에야 rclpy(ROS 2 파이썬 라이브러리)를 import할 수 있다. 시스템에 ROS 2를 따로 설치할 필요는 없다.
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped  # noqa: E402
from line_common import (  # noqa: E402
    AMMR_HOME,
    BELT_V,
    BELT_Z,
    BOX,
    PALLET_BUF,
    PALLET_OUT,
    PALLET_TOP,
    SCAN_LEN,
    SCAN_START,
    SLOT_X,
    TAKT,
    TOOLS,
    TUCK,
    X_IN,
    X_OUT,
    from_transform,
    seconds,
    stamp,
    to_transform,
)
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

# ------------------------------------------------------------------ 시뮬레이터 쪽 상수
# 라인 배치(벨트·팔레트·박스 크기)와 tooldata는 컨트롤러와 같이 쓰므로 line_common.py에 있다.
# 여기에는 로봇을 장면에 놓고 처음 세우는 데만 쓰는 값을 둔다.
START_T = -2.0  # [s] 시작 시각. 첫 박스가 집히기 2초 전 상태에서 시작한다
DOWN = np.array([0.0, 0.0, -1.0])  # [단위 없음] (x, y, z) 아래를 가리키는 길이 1짜리 방향 벡터
KUKA_HOME = [0.0, 0.0, 0.3, 0.0, 1.27, 0.0]  # [rad] 관절 a1~a6의 시작 자세. 공구가 아래를 본다
KAWASAKI_HOME = [0.0, 0.2, -1.5, 0.0, -1.44, 0.0] + [0.0] * 6  # [rad] 관절 j1~j6의 시작 자세 + 그리퍼 관절 6개
H = math.pi / 2  # [rad] 90도. 방향을 적을 때 쓰는 줄임말


# ------------------------------------------------------------------ 셀 상태와 센서
# 박스와 센서는 "공장 바닥"에 있는 것이라 시뮬레이터가 계산한다. 컨트롤러는 센서 값을 topic으로만 받는다.
io = {"doGrip": 0}  # 그리퍼 출력 신호. 컨트롤러가 /kawasaki/set_do_grip service로 바꾼다. 1이면 쥔 상태다
cell = {"held": None, "placed": {}, "log": []}  # held는 쥐고 있는 박스 번호, placed는 {박스 번호: 놓인 [m] 좌표}


def waiting_box(t):
    """벨트 끝에 도착해 집히기를 기다리는 박스의 번호를 돌려준다. 없으면 None이다."""
    for b in range(6):
        u = (t - b * TAKT) % (6 * TAKT)  # [s] 이 박스가 벨트 끝에 도착한 뒤 흐른 시간
        arrived_in_this_run = t - u > -1e-6  # 시작 전에 이미 팔레트에 있던 박스는 제외한다
        if u < 3 * TAKT and arrived_in_this_run and b != cell["held"] and b not in cell["placed"]:
            return b
    return None


def box_at_pick():
    """벨트 끝에 박스가 와 있으면 True. 근접 센서 하나다. 매 스텝 /cell/di_box_at_pick topic으로 내보낸다."""
    return waiting_box(state["t"]) is not None


def box_at_scan():
    """벨트 위 박스가 검사 시작 위치(SCAN_START)를 막 지났으면 True. 매 스텝 /cell/di_box_at_scan으로 내보낸다."""
    for b in range(6):
        u = (state["t"] - b * TAKT) % (6 * TAKT)
        if u >= 3 * TAKT and 0.0 <= X_IN + BELT_V * (u - 3 * TAKT) - SCAN_START[0] < 0.05:
            return True
    return False


def box_positions_rapid(t, tcp_pos):
    """RAPID 방식일 때 박스 6개의 위치. tcp_pos는 [m] 지금 실제 TCP 위치다.

    벨트 이동과 시작 전에 적재된 박스 배치는 시각으로 계산한다. 파지·해제는 doGrip 신호만 따른다.
    """
    out = np.zeros((6, 3))
    for b in range(6):
        u = (t - b * TAKT) % (6 * TAKT)  # [s] 박스별 순환 시각. 로봇 명령의 진행 시각은 아니다.
        if u >= 3 * TAKT:  # 벨트 위 이동
            out[b] = [X_IN + BELT_V * (u - 3 * TAKT), 0.0, BELT_Z + BOX / 2]
        else:  # 시작 전에 이미 적재된 박스의 배치. 이번 실행의 파지·해제 상태는 아래에서 적용한다.
            slot = SLOT_X[round((t - u) / TAKT) % 3]
            out[b] = [PALLET_OUT[0] + slot, PALLET_OUT[1], PALLET_TOP + BOX / 2]
    under_tcp = tcp_pos - [0.0, 0.0, BOX / 2]  # [m] TCP가 박스 윗면에 있을 때의 박스 중심
    if io["doGrip"] and cell["held"] is None:  # 쥐는 신호가 켜졌다. 벨트 끝에서 기다리던 박스를 잡는다
        cell["held"] = waiting_box(t)
    elif not io["doGrip"] and cell["held"] is not None:  # 신호가 꺼졌다. 박스를 지금 자리에 내려놓는다
        cell["placed"][cell["held"]] = under_tcp
        cell["log"].append(under_tcp)
        cell["held"] = None
    for b in range(6):
        u = (t - b * TAKT) % (6 * TAKT)
        if u >= 3 * TAKT:  # 벨트로 돌아간 박스는 놓였던 자리를 잊는다
            cell["placed"].pop(b, None)
        elif b in cell["placed"]:  # 놓인 자리에 그대로 있다
            out[b] = cell["placed"][b]
        elif b == cell["held"]:  # TCP에 붙어 따라간다
            out[b] = under_tcp
        elif t - u > -1e-6:  # 벨트 끝에 도착해 집히기를 기다린다
            out[b] = [X_OUT, 0.0, BELT_Z + BOX / 2]
    return out


def joint_step(arm, jac, error):
    """손끝 오차를 줄이는 관절 증분을 구해 위치 목표로 보낸다 (감쇠 최소제곱). 두 IK 함수가 같이 쓴다.

    jac은 6x6 자코비안, error는 [m] 위치 오차 3개와 [rad] 방향 오차 3개다.
    """
    dq = jac.T @ np.linalg.solve(jac @ jac.T + 0.01 * np.eye(6), error)
    q = arm["robot"].get_dof_positions().numpy()[0, :6]
    # 관절 지령을 스텝당 0.01 rad (= 2 rad/s) 이내로 적분한다. 실제 관절보다 0.1 rad 넘게 앞서가지 않게 묶는다.
    arm["q_cmd"] = np.clip(arm["q_cmd"] + np.clip(0.3 * dq, -0.01, 0.01), q - 0.1, q + 0.1)
    # 실제 Isaac Sim 제어 API를 호출하는 지점이다. 앞의 계산으로 얻은 각도를 첫 6개 팔 관절의 목표로 보낸다.
    # 위치 자체를 덮어쓰는 순간이동이 아니다. PhysX의 관절 구동기가 게인·감쇠에 따라 이 목표를 따라간다.
    arm["robot"].set_dof_position_targets(arm["q_cmd"], dof_indices=list(range(6)))


def link_pose(arm):
    """손끝 링크의 지금 실제 자세를 ([m] 위치, 방향)으로 돌려준다."""
    pos, quat = arm["ee"].get_world_poses()
    w, x, y, z = quat.numpy()[0]
    return pos.numpy()[0], Rotation.from_quat([x, y, z, w])  # Isaac Sim은 (w, x, y, z), scipy는 (x, y, z, w) 순서다


def ik_step_pose(arm, tool, target_pos, target_rot, link_pos, link_rot, t):
    """TCP를 목표 위치와 방향으로 보내는 미분 IK 한 스텝. RAPID 방식 명령이 쓴다.

    tooldata로 정한 TCP의 위치와 방향 전체 (rx, ry, rz)를 맞춘다.
    link_pos, link_rot은 link_pose(arm)으로 읽은 손끝 링크의 지금 실제 자세다.

    target_pos/target_rot은 MoveL이 이번 스텝까지 진행시킨 지령이다. 최종 robtarget까지 한 번에 점프하지 않는다.
    현재 TCP와 지령의 차이(error)를 만들고, 자코비안으로 필요한 관절 변화량을 계산한다.
    마지막 joint_step()이 관절 목표를 실제 API에 보내므로 이 함수까지 호출되어야 로봇이 움직인다.
    """
    reach = link_rot.apply(tool["pos"])  # [m] 손끝 링크 원점에서 TCP까지의 벡터 (월드 기준)
    tcp_pos, tcp_rot = link_pos + reach, link_rot * tool["rot"]
    error = np.concatenate([target_pos - tcp_pos, (target_rot * tcp_rot.inv()).as_rotvec()])
    jac = arm["robot"].get_jacobian_matrices().numpy()[0, arm["row"], :, :6].copy()
    jac[:3] -= np.cross(reach, jac[3:].T).T  # 링크 원점의 속도를 TCP의 속도로 바꾼다 (v_tcp = v + w x reach)
    joint_step(arm, jac, error)
    if t > 3.0:  # 자체 점검용 기록 (초기 수렴 구간 제외)
        arm["errors"].append(np.linalg.norm(target_pos - tcp_pos))
        arm["aligns"].append(float(tcp_rot.apply([0.0, 0.0, 1.0]) @ DOWN))


def h1_pose():
    """H1 몸체의 지금 실제 자세를 ([m] x, [m] y, [rad] 바라보는 방향)으로 돌려준다."""
    pos, quat = h1.articulation.get_world_poses()
    (x, y, _), (qw, qx, qy, qz) = pos.numpy()[0], quat.numpy()[0]
    return x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


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
add("/Isaac/Robots/Clearpath/RidgebackFranka/ridgeback_franka.usd", "/World/AMMR", (*AMMR_HOME, 0.0))


def make_arm(prim_path, ee_link, home):
    """IK로 움직일 팔 하나를 준비해 필요한 것들을 한 묶음(dict)으로 돌려준다.

    prim_path  add()에서 붙인 로봇 경로.
    ee_link    손끝으로 쓸 링크 이름. robot.link_names를 출력해서 고른다.
    home       [rad] 시작 자세. 로봇의 전체 관절 수만큼 적는다.
    """
    # Articulation은 관절로 이어진 로봇 한 대를 다루는 손잡이다. 관절 각도를 읽고 목표를 보낼 때 쓴다.
    robot = Articulation(prim_path)
    robot.set_default_state(dof_positions=home)
    return {
        "robot": robot,
        "ee": RigidPrim(f"{prim_path}/{ee_link}"),
        "ee_link": ee_link,  # /kuka/flange_pose 같은 메시지의 child_frame_id에 이 이름을 적는다
        "row": robot.get_link_indices(ee_link).list()[0] - 1,  # 고정 베이스라 자코비안에 루트 링크가 없다
        "home": np.array(home[:6], dtype=float),
        "q_cmd": np.array(home[:6], dtype=float),  # 적분 중인 관절 지령
        "errors": [],
        "aligns": [],
    }


# ------------------------------------------------------------------ 로봇을 제어 대상으로 잡기
# add()는 모델을 장면에 놓기만 한다. 관절을 읽고 움직이려면 Articulation(경로)으로 잡아야 한다.
# 내 로봇을 넣을 때 정해야 하는 것은 아래 세 가지다. 공구 방향은 tooldata에 정의한다.
#   경로        add()에서 붙인 이름.
#   손끝 링크    robot.link_names를 출력해 공구가 달린 링크 이름을 고른다.
#   시작 자세    [rad] 관절 한계 안에서 팔꿈치가 굽은 자세로 정한다. 팔이 곧게 펴진 자세는 IK가 불안정하다.
kuka = make_arm("/World/Kuka", "tool0", KUKA_HOME)  # 손끝은 tool0 링크. 공구 축은 line_common.TOOLS의 tScanner에서 정의한다.
kawasaki = make_arm("/World/Kawasaki", "gripper_center", KAWASAKI_HOME)  # 그리퍼 중심 링크. 공구 축은 tGripper에서 정의한다.
# AMMR은 MoveBase·MoveAbsJ가 관절 목표를 직접 만들므로 Articulation만 만든다.
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

# ------------------------------------------------------------------ 좌표 찍기 도구
# RobotStudio에서는 물체 윗면에 스냅을 걸고 클릭해서 타깃을 찍는다. 이 예시에서 같은 일을 하는 방법은 두 가지다.
#   1. 실행 중에 물체를 클릭한다        터미널에 "[TEACH] robtarget(...)" 한 줄이 찍힌다. 그 줄을 복사해
#                                      rapid_controller.py의 목표 등록부에 붙인다. pPalletTop이 그 예다.
#   2. 숫자를 직접 적는다               벨트 끝처럼 물체가 아니라 "라인에서 정한 위치"일 때 쓴다. pPick이 그 예다.
# 목표는 컨트롤러 파일에 있다. 컨트롤러는 장면을 볼 수 없으므로, 장면에서 좌표를 읽는 snap_top은 이 파일에만 있다.
stage = stage_utils.get_current_stage()


def snap_top(prim_path, rx=180.0, ry=0.0, rz=0.0):
    """물체의 "윗면 가운데"를 robtarget으로 돌려준다. RobotStudio에서 Snap Center로 물체 윗면을 찍는 것에 해당한다.

    prim_path는 Stage 창에 보이는 경로다. 물체를 감싸는 상자(바운딩 박스)의 윗면 중심을 쓴다.
    (rx, ry, rz)는 [deg] 그 점에서의 공구 방향이고, 기본값은 공구가 바닥을 보는 자세다.
    형상이 없는 프림(빈 Xform, 조명)이면 None을 돌려준다.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if box.IsEmpty():
        return None
    low, high = box.GetMin(), box.GetMax()
    return robtarget((low[0] + high[0]) / 2, (low[1] + high[1]) / 2, high[2], rx, ry, rz)


# ------------------------------------------------------------------ 실행
# 실행 중에 바뀌는 값들. t는 [s] 시뮬레이션 시각, first는 "재생 후 첫 스텝인가", error는 물리 콜백 오류다.
# ammr_x는 방금 AMMR에 보낸 [m] 베이스 x 목표, boxes는 방금 계산한 박스 6개의 [m] 좌표,
# taught는 마지막으로 클릭해서 찍은 (물체 경로, [m] 좌표)다. ages는 [s] 적용한 팔 지령이 만들어진 뒤 흐른 시간들이다.
state = {"t": START_T, "first": True, "error": None, "ammr_x": 0.0, "boxes": None, "taught": None, "ages": [], "received": 0}

# ------------------------------------------------------------------ ROS 2 노드 (/isaac_sim)
# 이 프로세스도 ROS 2 노드 하나가 된다. 노드는 "ROS 2 네트워크에 이름을 걸고 참여한 프로그램 하나"다.
# 여기서 만드는 것은 세 종류다.
#   발행(publisher)    로봇 상태와 센서 값을 매 물리 스텝 내보낸다. 받는 쪽이 누구인지는 모른다
#   구독(subscription) 컨트롤러가 보낸 지령을 받는다. 받은 지령은 cmd에 저장해 두고 물리 콜백이 꺼내 쓴다
#   서비스(service)    SetDO doGrip 요청을 받아 그리퍼 신호를 바꾸고 응답한다
# topic 이름과 메시지 타입은 rapid_controller.py 맨 위 목록과 같아야 한다. 하나라도 다르면 연결되지 않는다.
# 내 로봇을 추가하면 그 로봇의 상태 발행 하나와 지령 구독 하나를 같은 모양으로 추가한다.
rclpy.init()
ros = rclpy.create_node("isaac_sim")
cmd = {}  # 마지막으로 받은 지령. 키는 로봇 이름 (kuka, kawasaki, ammr, h1)


def keep(name):
    """받은 메시지를 cmd[name]에 저장하는 콜백을 만든다. 구독 콜백은 저장만 하고, 적용은 물리 콜백이 한다."""
    def store(msg):
        cmd[name] = msg
        state["received"] += 1  # 받기 반복이 "더 처리할 메시지가 없다"를 알아채는 데 쓴다

    return store


ros.create_subscription(TransformStamped, "/kuka/tcp_command", keep("kuka"), 10)
ros.create_subscription(TransformStamped, "/kawasaki/tcp_command", keep("kawasaki"), 10)
ros.create_subscription(JointState, "/ammr/joint_command", keep("ammr"), 10)
ros.create_subscription(TwistStamped, "/h1/cmd_vel", keep("h1"), 10)


def on_set_do_grip(request, response):
    """SetDO doGrip 요청을 처리한다. request.data가 True면 쥔다. 처리 결과를 response에 담아 돌려준다."""
    io["doGrip"] = int(request.data)
    state["received"] += 1
    response.success = True
    return response


ros.create_service(SetBool, "/kawasaki/set_do_grip", on_set_do_grip)
pub = {
    "clock": ros.create_publisher(Clock, "/clock", 10),
    "kuka": ros.create_publisher(TransformStamped, "/kuka/flange_pose", 10),
    "kawasaki": ros.create_publisher(TransformStamped, "/kawasaki/flange_pose", 10),
    "ammr": ros.create_publisher(JointState, "/ammr/joint_states", 10),
    "h1": ros.create_publisher(PoseStamped, "/h1/pose", 10),
    "pick": ros.create_publisher(Bool, "/cell/di_box_at_pick", 10),
    "scan": ros.create_publisher(Bool, "/cell/di_box_at_scan", 10),
}
# 실행기(executor)는 도착한 메시지를 확인해 위 콜백을 불러 주는 장치다. 물리 콜백 안에서 한 번씩 돌린다.
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(ros)

# 컨트롤러 노드를 별도 프로세스로 띄운다. 이 Python과 같은 실행 파일(sys.executable)과 같은 환경 변수를 쓴다.
# 실제 공장이라면 다른 PC에서 도는 프로그램이다. 같은 PC의 다른 프로세스여도 ROS 2 입장에서는 차이가 없다.
# --no-controller면 띄우지 않는다. 그때는 다른 터미널에서 python.bat rapid_controller.py를 직접 실행한다.
here = os.path.dirname(os.path.abspath(__file__))
controller = None
if not args.no_controller:
    # stdin 파이프는 이 프로세스가 어떻게 끝나든(Ctrl+C·예외·강제 종료) 닫힌다. 컨트롤러는 그것을 보고 따라 끝난다.
    controller = subprocess.Popen([sys.executable, os.path.join(here, "rapid_controller.py"), "--child"], cwd=here, stdin=subprocess.PIPE)
    print(f"[ROS2] started /rapid_controller (pid {controller.pid})", flush=True)


def fresh(msg):
    """이번 실행에서 만들어진 지령이면 True.

    정지 후 재생하면 시계가 0으로 돌아간다. 그보다 미래 시각이 찍힌 지령은 이전 실행에서 늦게 도착한 것이라 버린다.
    """
    return msg is not None and seconds(msg.header.stamp) <= state["t"] - START_T + 1e-6


# ------------------------------------------------------------------ 클릭으로 좌표 찍기 (티칭)
# 뷰포트나 Stage 창에서 물체를 클릭하면 그 물체 윗면 가운데 좌표가 robtarget 한 줄로 출력된다.
# 재생 중이든 일시정지 중이든 된다. 움직이는 박스는 일시정지(Ⅱ)한 뒤 클릭하면 그 순간의 위치가 찍힌다.
teach_marker = Cube("/World/TeachMarker", sizes=0.04, colors=[1.0, 0.1, 0.1], positions=[[0.0, 0.0, -1.0]])  # 찍힌 자리 표시


def on_stage_event(event):
    """선택이 바뀔 때마다 불린다. 선택한 물체의 윗면 가운데를 찍어 출력하고 빨간 표시를 옮긴다."""
    if event.type != int(omni.usd.StageEventType.SELECTION_CHANGED):
        return
    paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
    if not paths or paths[0] == "/World/TeachMarker":
        return
    target = snap_top(paths[0])
    if target is None:
        return
    x, y, z = target["pos"]
    line = f"robtarget({x:.3f}, {y:.3f}, {z:.3f}, 180.0, 0.0, 0.0)  # {paths[0]} 윗면 가운데"
    print("[TEACH] " + line, flush=True)  # 실행한 터미널에 찍힌다
    carb.log_warn("[TEACH] " + line)  # Isaac Sim 창의 Console 탭에도 찍힌다
    teach_marker.set_world_poses(positions=[target["pos"]])
    state["taught"] = (paths[0], target["pos"])


teach_subscription = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
    on_stage_event, name="factory_line_demo teach"
)


def on_physics_step(dt, context):
    """물리가 한 스텝(0.005 s) 진행될 때마다 Isaac Sim이 불러 주는 함수. ROS 2로 상태를 내보내고 지령을 적용한다.

    dt는 [s] 방금 진행된 물리 한 스텝의 길이다. context는 Isaac Sim이 넘겨주는 값이고 여기서는 쓰지 않는다.
    한 스텝에서 하는 일은 세 가지다.
      1. 내보내기   로봇 상태·센서 값을 발행하고 마지막에 /clock을 발행한다. 컨트롤러는 /clock을 받으면 프로그램을 진행한다
      2. 받기      그사이 도착한 지령 메시지를 실행기로 처리해 cmd에 저장한다
      3. 적용      cmd의 지령을 IK·관절 목표·보행 정책으로 로봇에 적용한다
    컨트롤러의 답은 보통 한두 스텝 뒤에 도착한다. 그동안은 직전 지령을 계속 적용한다.
    내 로봇을 추가했다면 1에 상태 발행을, 3에 지령 적용을, "첫 스텝" 부분에 시작 자세로 되돌리는 줄을 넣는다.
    """
    if state["error"]:  # 앞에서 오류가 났으면 더 진행하지 않는다
        return
    try:
        # --- 첫 스텝. 재생 직후 한 번만 실행된다 (정지 후 다시 재생했을 때도 여기로 온다).
        #     시계와 로봇 자세를 처음 상태로 맞추고 끝낸다. 컨트롤러는 시계가 되돌아간 것을 보고 프로그램을 처음부터 다시 시작한다.
        if state["first"]:
            state.update(t=START_T, first=False)
            # 관절 구동기의 게인을 정한다. 앞이 강성 kp, 뒤가 감쇠 kd다. kp가 클수록 목표각으로 세게 당긴다.
            # KUKA 에셋 기본 게인(5.7e16)에서는 지령 없이 a1이 0.13 rad 흘렀다. 다른 팔과 같은 값으로 맞춘다.
            # 내 로봇이 목표를 못 따라가거나 떨면 이 값을 조정한다.
            kuka["robot"].set_dof_gains(5.73e8, 5.73e6)
            for arm in (kuka, kawasaki):
                arm["q_cmd"] = arm["home"].copy()  # IK가 적분해 가는 지령 관절각도 시작 자세로 되돌린다
            cmd.clear()  # 이전 실행의 지령은 버린다. 새 지령이 올 때까지 로봇은 시작 자세를 유지한다
            io["doGrip"] = 0
            cell.update(held=None, placed={})
            for robot in (kuka["robot"], kawasaki["robot"], ammr):
                robot.reset_to_default_state()  # set_default_state로 정해 둔 자세로 한 번에 옮긴다
            # H1을 기본 자세로 세우고 보행 정책을 초기화한다. 인자는 속도 명령 [전진, 횡, 회전]이고 처음에는 0이다.
            h1.restart_from_default_state(np.zeros(3, dtype=np.float32))
            return
        state["t"] += dt
        t = state["t"]
        now = t - START_T  # [s] 재생을 시작한 뒤 흐른 시간. /clock과 메시지 시각에 쓴다 (ROS 시각은 0부터 센다)
        # --- 1. 내보내기. 로봇의 지금 실제 상태와 센서 값이다.
        arm_pose = {}
        for arm, name in ((kuka, "kuka"), (kawasaki, "kawasaki")):
            arm_pose[name] = link_pose(arm)  # 손끝 링크의 지금 실제 자세 ([m] 위치, 방향)
            pub[name].publish(to_transform(now, *arm_pose[name], f"{name}/{arm['ee_link']}"))
        q = ammr.get_dof_positions().numpy()[0]  # AMMR 관절 12개. 0~2가 베이스 가상 관절, 3~9가 팔 7축
        joints = JointState(name=list(ammr.dof_names[:10]), position=[float(v) for v in q[:10]])
        joints.header.stamp = stamp(now)
        pub["ammr"].publish(joints)
        x, y, yaw = h1_pose()
        h1_msg = PoseStamped()
        h1_msg.header.stamp, h1_msg.header.frame_id = stamp(now), "world"
        h1_msg.pose.position.x, h1_msg.pose.position.y = float(x), float(y)
        h1_msg.pose.orientation.z, h1_msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        pub["h1"].publish(h1_msg)
        pub["pick"].publish(Bool(data=box_at_pick()))
        pub["scan"].publish(Bool(data=box_at_scan()))
        pub["clock"].publish(Clock(clock=stamp(now)))  # 마지막에 보낸다. 컨트롤러는 이것을 받으면 위 값들로 한 스텝 진행한다
        # --- 2. 받기. 도착해 있는 메시지를 모두 처리한다. timeout_sec=0이라 기다리지 않는다.
        #     먼저 방금 보낸 /clock에 대한 두 팔의 답을 기다린다([s] 최대 0.005). 안 기다리면 시뮬레이션이 컨트롤러보다
        #     앞서 나가 지령이 100 ms 넘게 늦어지고 팔이 목표를 놓친다 (2026-11-23 60초 실행에서 확인).
        #     spin_once는 할 일이 없어도 한 번에 약 0.2 ms가 든다. 그 뒤는 처리한 메시지가 없으면 바로 멈춘다.
        deadline = time.perf_counter() + 0.005
        while controller and time.perf_counter() < deadline and not all(
            cmd.get(n) and seconds(cmd[n].header.stamp) >= now - 1e-6 for n in ("kuka", "kawasaki")
        ):
            executor.spin_once(timeout_sec=0.0)
        for _ in range(20):
            before = state["received"]
            executor.spin_once(timeout_sec=0.0)
            if state["received"] == before:
                break
        # --- 3. 적용. 팔 두 대는 지령 TCP 자세로 IK를 한다. tool은 메시지에 실려 온 이름으로 찾는다.
        for arm, name in ((kuka, "kuka"), (kawasaki, "kawasaki")):
            msg = cmd.get(name)
            if fresh(msg):
                state["ages"].append(now - seconds(msg.header.stamp))
                ik_step_pose(arm, TOOLS[msg.child_frame_id], *from_transform(msg), *arm_pose[name], t)
            else:  # 지령이 아직 없다. 지금 지령 관절각을 그대로 유지한다
                arm["robot"].set_dof_position_targets(arm["q_cmd"], dof_indices=list(range(6)))
        # AMMR. 베이스 가상 관절 3개 + 팔 7축의 목표를 그대로 관절 구동기에 보낸다. 손가락 2개는 건드리지 않는다.
        # MoveBase는 차체용 가상 관절 3개, MoveAbsJ는 Panda 팔 7개에 대응한다. 바퀴 속도 제어는 아니다.
        msg = cmd.get("ammr")
        targets = list(msg.position[:10]) if fresh(msg) else [0.0, 0.0, 0.0, *TUCK]
        ammr.set_dof_position_targets(targets, dof_indices=list(range(10)))
        state["ammr_x"] = targets[0]
        # H1. 속도 명령 [전진 m/s, 횡 m/s, 회전 rad/s]을 보행 정책에 넘긴다. 관절 19개의 목표는 정책이 계산한다.
        msg = cmd.get("h1")
        vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z] if fresh(msg) else [0.0, 0.0, 0.0]
        h1.step(dt, np.array(vel, dtype=np.float32))
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
scan_off, scan_n = 0.0, 0  # RAPID 방식에서 KUKA가 박스와 나란히 갈 때의 [m] 최대 어긋남과 잰 프레임 수
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
    # 박스 6개를 이 시각의 위치로 옮긴다. 두 번째 인자는 [m] Kawasaki 그리퍼의 현재 위치이고,
    # 집힌 박스가 그 위치를 따라가는 데 쓰인다.
    link_pos, link_rot = link_pose(kawasaki)
    state["boxes"] = box_positions_rapid(state["t"], link_pos + link_rot.apply(TOOLS["tGripper"]["pos"]))
    boxes.set_world_poses(positions=state["boxes"])
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
    ammr_err = max(ammr_err, abs(ammr_q[0] - state["ammr_x"]))  # 베이스 x가 방금 보낸 목표에서 가장 멀어진 거리
    ammr_x_min = min(ammr_x_min, ammr_q[0])
    # KUKA가 박스 위를 나란히 가는 구간에서, TCP와 가장 가까운 벨트 위 박스의 x 차이를 잰다.
    scan_pos, scan_rot = link_pose(kuka)
    scan_tcp = scan_pos + scan_rot.apply(TOOLS["tScanner"]["pos"])
    low_enough = scan_tcp[2] < SCAN_START[2] + 0.02
    if low_enough and SCAN_START[0] + 0.05 < scan_tcp[0] < SCAN_START[0] + SCAN_LEN - 0.05:
        on_belt = [b for b in state["boxes"] if abs(b[1]) < 0.01 and abs(b[2] - (BELT_Z + BOX / 2)) < 0.01]
        scan_off = max(scan_off, min(abs(b[0] - scan_tcp[0]) for b in on_belt))
        scan_n += 1
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
    # errors는 ik_step_pose가 스텝마다 쌓아 둔 [m] 손끝 목표와 실제 위치의 거리다.
    # aligns는 공구 축과 아래 방향 사이 각도의 코사인이다. 1이면 정확히 아래를 본다.
    checks = [
        ("KUKA mean tip error < 0.03 m", np.mean(kuka["errors"]) < 0.03, f"mean {np.mean(kuka['errors']):.4f} max {max(kuka['errors'], default=math.inf):.4f}"),
        ("KUKA tool points down", np.mean(kuka["aligns"]) > 0.95, f"mean cos {np.mean(kuka['aligns']):.3f}"),
        ("Kawasaki mean tip error < 0.03 m", np.mean(kawasaki["errors"]) < 0.03, f"mean {np.mean(kawasaki['errors']):.4f} max {max(kawasaki['errors'], default=math.inf):.4f}"),
        ("Kawasaki tool points down", np.mean(kawasaki["aligns"]) > 0.95, f"mean cos {np.mean(kawasaki['aligns']):.3f}"),
        ("AMMR base follows its path (< 0.3 m)", ammr_err < 0.3, f"max lag {ammr_err:.3f} m, reached x {ammr_x_min:.2f} m"),
        ("H1 stays upright (pelvis > 0.8 m)", h1_z_min > 0.8, f"min z {h1_z_min:.3f} m"),
        ("H1 keeps walking (> 0.25 m/s)", h1_travel / elapsed > 0.25, f"{h1_travel:.1f} m in {elapsed:.0f} s"),
    ]
    # RAPID 방식 전용 항목. 프로그램이 박스를 제때 집어 팔레트 칸에 정확히 놓았는지 본다.
    slots = [np.array([PALLET_OUT[0] + dx, PALLET_OUT[1], PALLET_TOP + BOX / 2]) for dx in SLOT_X]
    offs = [min(float(np.linalg.norm(placed - slot)) for slot in slots) for placed in cell["log"]]  # [m] 칸 중심과의 거리
    expected = int(max(state["t"] - 5.0, 0.0) // TAKT) + 1  # 박스는 TAKT마다 오고, 집은 뒤 5초 안에 놓여야 한다
    # ROS 2 연결 항목. 컨트롤러 지령이 실제로 도착해 적용됐는지 본다. age는 지령이 만들어진 뒤 적용될 때까지 걸린
    # [s] 시뮬레이션 시간이다. 0.005가 물리 한 스텝이다.
    ages = state["ages"]
    checks.append(
        ("ROS 2 commands from /rapid_controller were applied", len(ages) > 0, f"{len(ages)} arm commands, mean age {np.mean(ages or [math.inf]) * 1000:.1f} ms")
    )
    checks += [
        ("RAPID program kept up with the takt", len(offs) >= expected, f"placed {len(offs)} boxes, expected {expected}"),
        ("RAPID program placed boxes on slots (< 0.01 m)", bool(offs) and max(offs) < 0.01, f"max offset {max(offs, default=math.inf):.4f} m"),
        ("KUKA stays over the box while scanning (< 0.03 m)", scan_n > 0 and scan_off < 0.03, f"max x offset {scan_off:.4f} m over {scan_n} frames"),
    ]
    realtime = elapsed / (time.time() - wall_start)  # 1이면 실시간, 0.7이면 실제 1초에 시뮬레이션 0.7초
    # --- 클릭 티칭 점검. 팔레트를 선택했을 때 그 윗면 가운데가 찍히는지 본다. GUI에서 클릭하는 것과 같은 이벤트다.
    selection = omni.usd.get_context().get_selection()
    selection.set_selected_prim_paths(["/World/PalletOut"], True)
    for _ in range(3):
        frame()
    selection.set_selected_prim_paths([], True)
    taught = state["taught"]
    taught_ok = taught is not None and taught[0] == "/World/PalletOut"
    taught_off = float(np.linalg.norm(taught[1] - [PALLET_OUT[0], PALLET_OUT[1], PALLET_TOP])) if taught_ok else math.inf
    checks.append(("Click-to-teach reports the top centre of the selected object", taught_off < 0.01, f"offset {taught_off:.4f} m"))
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
if controller is not None:  # 같이 띄운 컨트롤러 노드를 끝낸다
    controller.terminate()
    controller.wait(timeout=10)
ros.destroy_node()  # 이 프로세스의 ROS 2 노드를 네트워크에서 내린다
rclpy.shutdown()
simulation_app.close()  # Isaac Sim 프로그램 자체를 끝낸다
