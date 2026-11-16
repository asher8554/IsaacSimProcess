# 산업용 로봇 2대·휴머노이드·AMMR이 한 라인에서 움직이는 Isaac Sim 6.2 공장 자동화 모션 예시
r"""실행: C:\isaacsim-6.2\python.bat factory_line_demo.py [--headless] [--seconds 40]

- 산업용 로봇 A (KUKA KR210)       컨베이어 위를 지나가는 박스를 따라가며 검사하는 동작
- 산업용 로봇 B (Kawasaki RS080N)  컨베이어 끝에서 박스를 들어 팔레트에 내려놓는 동작
- 휴머노이드 (Unitree H1)          라인 옆 통로를 순찰 보행 (Isaac Sim 내장 보행 정책)
- AMMR (Ridgeback + Franka)        팔레트와 출하 버퍼 사이를 왕복하며 팔을 뻗는 동작

ROS 2는 쓰지 않는다. 박스는 물리 없는 시각용 큐브이고 그리퍼 위치를 따라다닐 뿐이다.

로봇 동작은 두 가지 방식으로 적혀 있고 --motion으로 고른다. 장면과 박스는 같다.
- --motion rapid (기본)  ABB RAPID처럼 "목표·속도·zone·tool"로 적은 방식. 네 로봇 모두 프로그램으로 움직인다.
                         scan_program, palletize_program, ammr_program, patrol_program 참고.
- --motion time          "몇 초에 어디"를 적은 이전 시간표 방식. scan_target, palletize_target, AMMR_KEYS 참고.

이 파일이 SimulationApp을 생성하므로 Isaac Sim을 별도로 열 필요가 없다.
기본 실행의 제어 흐름은 다음과 같다. 명령은 설명용 예시가 아니라 매 물리 스텝의 구동에 쓰인다.
    *_program()의 yield 명령 → RapidTask.step()의 지령 → on_physics_step()의 제어 → Isaac Sim 관절 구동기
팔 두 대는 IK로, AMMR은 관절 목표로, H1은 보행 정책으로 같은 RAPID 스타일 명령을 실행한다.

실행 중에 물체를 클릭하면 그 물체 윗면 가운데 좌표가 robtarget 한 줄로 출력된다 (on_stage_event 참고).
"""

import argparse
import os

# 이 스크립트의 수치 계산은 6x6 이하의 작은 행렬뿐이다. numpy·scipy·torch가 계산용 스레드를 여러 개 띄우면
# 도움은 안 되고 물리·화면 스레드와 CPU를 다투기만 한다. 1개로 묶으면 RAPID 방식이 실시간 대비 0.32배에서
# 0.47배로, 시간표 방식이 0.68배에서 0.75배로 빨라졌다 (14초 헤드리스 실측). Isaac Sim을 띄우기 전에 정해야 적용된다.
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
    "--motion",
    choices=["time", "rapid"],
    default="rapid",
    help="네 로봇의 구동 방식. rapid(기본)는 RAPID 스타일 프로그램, time은 이전 시간표 방식",
)
args, _ = parser.parse_known_args()
# 별도 Isaac Sim 프로세스에 명령을 보내는 구조가 아니다. 이 Python 실행 안에서 앱과 물리 엔진을 시작한다.
print(f"[MOTION] {args.motion} | " + ("RAPID programs -> robot drives" if args.motion == "rapid" else "legacy timetable -> robot drives"), flush=True)
simulation_app = SimulationApp({"headless": args.headless})

import math
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
import omni.usd
from pxr import Usd, UsdGeom
from rapid_style import (
    MoveAbsJ,
    MoveBase,
    MoveL,
    Offs,
    RapidTask,
    SetDO,
    WaitTime,
    WaitUntil,
    basetarget,
    fine,
    jointtarget,
    robtarget,
    tooldata,
    v500,
    v1000,
    v1500,
    z50,
    z500,
)
from scipy.spatial.transform import Rotation

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
AMMR_HOME = (PALLET_OUT[0], -3.0)  # [m] (x, y) AMMR을 처음 놓는 위치. 적재 팔레트 앞 통로다
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
    """로봇 A(KUKA)의 손끝이 시각 t에 있어야 할 [m] 월드 좌표 (x, y, z)를 돌려준다.

    택트(8초)마다 같은 동작을 되풀이한다. 벨트 위를 지나가는 박스 바로 위로 내려가 박스와 같은 속도로
    함께 이동하고(검사하는 동작), 다시 올라와 다음 박스가 올 자리로 돌아간다.
    이 함수는 "지금 손끝이 어디 있어야 하는가"만 정한다. 로봇을 그 좌표로 보내는 일은 ik_step이 한다.

    이것은 컨베이어 동기화(컨베이어 트래킹)가 아니다. 로봇은 박스나 벨트의 위치를 읽지 않는다. 박스 위치와
    손끝 목표가 둘 다 같은 시계 t로 계산되고, 박스가 지나가는 시각에 맞춰 시간표를 짜 두었을 뿐이다.
    BELT_V를 바꾸면 아래 시간표도 같이 고쳐야 맞는다.

    손끝이 거쳐 가는 네 지점과 순서 (옆에서 본 그림. 박스는 왼쪽에서 오른쪽으로 흐른다).

        z
        ^   start_up o <--------(4)--------- o end_up
        |            |                       ^
        |           (1)                     (3)
        |            v                       |
        |      start o ---------(2)--------> o end
        +----------------------------------------------> x
               x = -1.45                  x = -0.55

        (1) 박스 위로 내려간다  (2) 박스와 함께 이동한다  (3) 올라간다  (4) 다음 박스 자리로 돌아간다
    """
    # --- 1. 손끝이 거쳐 가는 네 지점. 모두 [m] 월드 좌표 (x, y, z)다.
    x_start, x_end = -1.45, -0.55  # [m] 박스를 따라가기 시작하는 x와 끝내는 x. 벨트 위 0.9 m 구간이다
    low = BELT_Z + BOX + 0.10  # [m] 따라갈 때의 손끝 높이. 박스 윗면에서 10 cm 위
    high = BELT_Z + BOX + 0.50  # [m] 기다리거나 돌아갈 때의 손끝 높이. 박스 윗면에서 50 cm 위
    start_up = [x_start, 0.0, high]
    start = [x_start, 0.0, low]
    end = [x_end, 0.0, low]
    end_up = [x_end, 0.0, high]

    # --- 2. 시간표. 한 줄이 ([s] 택트가 시작된 뒤의 시각, 그 시각에 손끝이 있을 지점)이다.
    # 줄과 줄 사이에서는 앞 지점에서 뒤 지점으로 이동한다. 앞뒤 지점이 같으면 그 자리에 멈춰 있다.
    schedule = [
        (0.0, start_up),  # 높은 곳에서 다음 박스를 기다린다
        (1.2, start_up),  # 1.2초 동안 멈춰 있다
        (2.0, start),  # 0.8초 동안 (1) 내려간다. 다 내려온 순간 박스가 바로 아래에 와 있다
        (6.0, end),  # 4.0초 동안 (2) 박스와 함께 이동한다. 0.9 m / 4.0 s = 벨트 속도 0.225 m/s
        (6.8, end_up),  # 0.8초 동안 (3) 올라간다
        (TAKT, start_up),  # 1.2초 동안 (4) 돌아간다. 여기서 택트가 끝나고 맨 윗줄로 돌아간다
    ]

    # --- 3. 시간표에서 지금 시각에 해당하는 좌표를 계산한다. t를 택트로 나눈 나머지는 path()가 구한다.
    # smooth=False는 구간을 일정한 속도로 이동하라는 뜻이다. (2) 구간에서 박스와 속도가 정확히 같아야 하므로
    # 출발·도착 때 느려지는 부드러운 보간을 쓰지 않는다.
    return path(schedule, t, smooth=False)


def palletize_target(t):
    """로봇 B(Kawasaki)의 손끝이 시각 t에 있어야 할 [m] 월드 좌표 (x, y, z)를 돌려준다.

    택트(8초)마다 같은 동작을 되풀이한다. 벨트 끝에서 박스를 집어 팔레트의 한 칸에 놓고 돌아온다.
    이 함수는 "지금 손끝이 어디 있어야 하는가"만 정한다. 로봇을 그 좌표로 보내는 일은 ik_step이,
    박스를 그리퍼에 붙였다 떼는 일은 box_positions가 따로 한다.

    손끝이 거쳐 가는 네 지점과 순서 (옆에서 본 그림).

        z
        ^      pick_up o ---------(2)---------> o place_up
        |              ^                        |
        |             (1)                      (3)
        |              |                        v
        |         pick o                        o place
        +------------------------------------------------>
               벨트 끝                         팔레트 칸

        (1) 들어 올린다  (2) 팔레트 위로 옮긴다  (3) 내린다  → 박스를 놓는다
        그다음 빈손으로 (3) (2) (1)을 거꾸로 되돌아와 다음 박스를 기다린다.
    """
    # --- 1. 손끝이 거쳐 가는 네 지점. 모두 [m] 월드 좌표 (x, y, z)다.
    # 집는 점. 벨트 끝에 도착한 박스의 윗면에서 GAP만큼 위다.
    pick = np.array([X_OUT, 0.0, BELT_Z + BOX + GAP])
    # 이번 택트에 박스를 놓을 칸 번호. 택트가 바뀔 때마다 0 → 1 → 2 → 0 순서로 돈다.
    slot = int(t // TAKT) % 3
    # 놓는 점. 그 칸에 놓인 박스의 윗면에서 GAP만큼 위다.
    place = np.array([PALLET_OUT[0] + SLOT_X[slot], PALLET_OUT[1], PALLET_TOP + BOX + GAP])
    # 집는 점과 놓는 점의 바로 위. 박스를 옆으로 옮길 때는 이 높이까지 들어서 벨트와 다른 박스를 피한다.
    pick_up = pick + [0.0, 0.0, 0.45]
    place_up = place + [0.0, 0.0, 0.60]

    # --- 2. 시간표. 한 줄이 ([s] 택트가 시작된 뒤의 시각, 그 시각에 손끝이 있을 지점)이다.
    # 줄과 줄 사이에서는 앞 지점에서 뒤 지점으로 부드럽게 이동한다. 앞뒤 지점이 같으면 그 자리에 멈춰 있다.
    schedule = [
        (0.0, pick),  # 박스가 벨트 끝에 도착하는 순간. 손끝은 이미 집는 점에 와 있다
        (0.4, pick),  # 0.4초 동안 멈춰 있다 (집는 시간)
        (1.4, pick_up),  # 1.0초 동안 (1) 들어 올린다
        (3.4, place_up),  # 2.0초 동안 (2) 팔레트 칸 위로 옮긴다
        (4.4, place),  # 1.0초 동안 (3) 내린다
        (RELEASE + 0.2, place),  # 0.4초 동안 멈춰 있다. 그 한가운데인 RELEASE(4.6초)에 박스를 놓는다
        (5.6, place_up),  # 0.8초 동안 빈손으로 올린다
        (7.3, pick_up),  # 1.7초 동안 벨트 끝 위로 돌아온다
        (TAKT, pick),  # 0.7초 동안 내려온다. 여기서 택트가 끝나고 맨 윗줄로 돌아간다
    ]

    # --- 3. 시간표에서 지금 시각에 해당하는 좌표를 계산한다. t를 택트로 나눈 나머지는 path()가 구한다.
    return path(schedule, t)


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


# ------------------------------------------------------------------ RAPID 프로그램 작성부 (기본 구동 방식)
# 위쪽 함수들은 "몇 초에 어디"를 적은 시간표다. 같은 동작을 ABB RAPID처럼
# "어디로(target), 얼마나 빨리(speed), 어떻게 지나서(zone), 무엇으로(tool)"라고 적은 것이 아래 프로그램들이다.
# 명령의 뜻과 ABB와 다른 점은 rapid_style.py 맨 위에 적혀 있다.
# 프로그램이 쓰는 목표(pPick, bOut 등)와 공구(tGripper 등)는 장면을 다 읽은 뒤에 등록한다.
# 아래쪽 "RAPID 방식의 목표·공구 등록" 구역에 있다.
# 동작을 바꾸려면 먼저 이 구역의 *_program()을 수정한다. scan_target/palletize_target은 --motion time 전용이다.
# 프로그램 작성자는 MoveL 등의 명령으로 순서를 적는다. 관절 계산과 물리 스텝 호출은 아래 실행부가 담당한다.

io = {"doGrip": 0}  # 출력 신호. doGrip이 1이면 그리퍼가 쥔 상태다
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
    """벨트 끝에 박스가 와 있으면 True. RAPID의 디지털 입력 diBoxAtPick에 해당한다."""
    return waiting_box(state["t"]) is not None


def box_at_scan():
    """벨트 위 박스가 검사 시작 위치(pScanStart)를 막 지났으면 True. RAPID의 디지털 입력에 해당한다."""
    for b in range(6):
        u = (state["t"] - b * TAKT) % (6 * TAKT)
        if u >= 3 * TAKT and 0.0 <= X_IN + BELT_V * (u - 3 * TAKT) - pScanStart["pos"][0] < 0.05:
            return True
    return False


# --- yield가 낯설다면
# 보통 함수는 return을 만나면 끝난다. yield가 들어 있는 함수는 다르다. yield를 만나면 그 값(명령 하나)을
# 내놓고 "그 자리에서 멈춰 기다린다". 다음에 다시 부르면 멈췄던 바로 다음 줄부터 이어서 실행한다.
# 그래서 아래 프로그램은 "명령을 한 줄씩 내놓는 목록"처럼 동작한다. 실행기 RapidTask가 명령 하나를 받아
# 로봇이 그 동작을 끝낼 때까지 여러 물리 스텝에 걸쳐 실행하고, 끝나면 다음 명령을 달라고 한다.
# RAPID에서 MoveL 한 줄이 끝나야 프로그램 포인터(PP)가 다음 줄로 넘어가는 것과 같다.
# while True 안에 있으므로 마지막 줄까지 가면 처음으로 돌아가 끝없이 되풀이한다 (WHILE TRUE DO ... ENDWHILE).
def palletize_program():
    """Kawasaki 적재 프로그램. 같은 내용을 RAPID로 쓰면 아래와 같다 (단위만 mm에서 m로 다르다).

        PROC Palletize()
          WHILE TRUE DO
            pPlace := Offs(pPallet, slotX{nSlot}, 0, 0);
            MoveL Offs(pPick, 0, 0, 450), v1500, z50, tGripper;
            MoveL pPick, v1000, fine, tGripper;
            WaitUntil diBoxAtPick = 1;
            SetDO doGrip, 1;
            WaitTime 0.3;
            MoveL Offs(pPick, 0, 0, 450), v1000, z50, tGripper;
            MoveL Offs(pPlace, 0, 0, 600), v1500, z50, tGripper;
            MoveL pPlace, v1000, fine, tGripper;
            SetDO doGrip, 0;
            WaitTime 0.3;
            MoveL Offs(pPlace, 0, 0, 600), v1000, z50, tGripper;
            nSlot := (nSlot + 1) MOD 3;
          ENDWHILE
        ENDPROC

    yield 한 줄이 RAPID 명령 한 줄이다. 시간표 방식과 달리 "몇 초에"가 없다. 걸리는 시간은 거리와 속도로 정해지고,
    박스가 오는 시각과는 WaitUntil로 맞춘다.

    이 함수 안의 PROC 예문은 설명이고, 아래의 실제 Python yield 문들이 로봇을 제어한다.
    예를 들어 MoveL의 pPick을 바꾸면 RapidTask의 직선 경로가 바뀌고, IK가 그 경로를 관절 목표로 바꾼다.
    fine에서 현재 TCP 위치가 목표의 5 mm 안에 들어온 뒤에만 다음 줄로 넘어간다.
    SetDO는 io 딕셔너리를 바꾸며 frame()의 box_positions_rapid가 그 값으로 박스를 붙이거나 놓는다.
    따라서 여기서 '쥔다'는 시각용 박스 연결이며 RG2 손가락의 물리 접촉 파지 명령은 아니다.
    """
    slot = 0  # 이번에 놓을 팔레트 칸 번호 (0, 1, 2)
    while True:
        pPlace = Offs(pPallet, SLOT_X[slot], 0.0, 0.0)  # 가운데 칸에서 x로 옮긴 이번 칸
        yield MoveL(Offs(pPick, 0.0, 0.0, 0.45), v1500, z50, tGripper)  # 집는 점 45 cm 위로 간다 (멈추지 않고 지나감)
        yield MoveL(pPick, v1000, fine, tGripper)  # 집는 점으로 내려가 정확히 선다
        yield WaitUntil(box_at_pick)  # 박스가 벨트 끝에 올 때까지 기다린다
        yield SetDO("doGrip", 1)  # 쥔다
        yield WaitTime(0.3)  # 쥐는 시간
        yield MoveL(Offs(pPick, 0.0, 0.0, 0.45), v1000, z50, tGripper)  # 들어 올린다
        yield MoveL(Offs(pPlace, 0.0, 0.0, 0.60), v1500, z50, tGripper)  # 놓을 칸 60 cm 위로 옮긴다
        yield MoveL(pPlace, v1000, fine, tGripper)  # 놓을 칸으로 내려가 정확히 선다
        yield SetDO("doGrip", 0)  # 놓는다
        yield WaitTime(0.3)  # 놓는 시간
        yield MoveL(Offs(pPlace, 0.0, 0.0, 0.60), v1000, z50, tGripper)  # 빈손으로 올린다
        slot = (slot + 1) % 3  # 다음 칸


def scan_program():
    """KUKA 검사 프로그램. 지나가는 박스 위 10 cm에서 박스와 나란히 이동한 뒤 돌아온다.

        PROC Scan()
          WHILE TRUE DO
            MoveL Offs(pScanStart, 0, 0, 400), v1000, z50, tScanner;
            MoveL pScanStart, v1000, fine, tScanner;
            WaitUntil diBoxAtScan = 1;
            MoveL pScanEnd, vBelt, fine, tScanner;
            MoveL Offs(pScanEnd, 0, 0, 400), v1000, z50, tScanner;
          ENDWHILE
        ENDPROC

    컨베이어 트래킹은 아니다. 박스가 시작 위치에 온 것을 신호로 받아 벨트와 같은 속도(vBelt)로 직선 이동할 뿐이다.
    """
    vBelt = BELT_V  # [m/s] 벨트와 같은 속도. speeddata를 직접 만든 것이다
    while True:
        yield MoveL(Offs(pScanStart, 0.0, 0.0, 0.40), v1000, z50, tScanner)  # 시작 위치 40 cm 위로 돌아온다
        yield MoveL(pScanStart, v1000, fine, tScanner)  # 박스가 지나갈 높이로 내려가 정확히 선다
        yield WaitUntil(box_at_scan)  # 박스가 바로 아래에 올 때까지 기다린다
        yield MoveL(pScanEnd, vBelt, fine, tScanner)  # 박스와 같은 속도로 나란히 간다
        yield MoveL(Offs(pScanEnd, 0.0, 0.0, 0.40), v1000, z50, tScanner)  # 올라간다


def ammr_program():
    """AMMR 프로그램. 적재 팔레트와 출하 버퍼 사이를 오가며 도착할 때마다 팔을 뻗었다 접는다.

    MoveBase가 몸체(바퀴 베이스)를, MoveAbsJ가 그 위의 팔 7축을 움직인다. 모두 fine이라 실제로 도착한 뒤에
    다음 줄로 넘어간다. MoveBase는 RAPID에 없는 명령이지만 모양은 MoveL과 같다 (목표, 속도, zone).
    제자리 회전은 x, y가 같고 rz만 다른 목표를 주면 된다.
    """
    while True:
        yield MoveBase(bOut, v500, fine)  # 적재 팔레트 앞에서 팔레트 쪽(+y)을 보고 선다
        yield MoveAbsJ(jReach, 45.0, fine)  # 팔을 앞으로 뻗는다. 45는 [deg/s]
        yield WaitTime(1.0)
        yield MoveAbsJ(jTuck, 45.0, fine)  # 팔을 접는다
        yield MoveBase(basetarget(AMMR_HOME[0], AMMR_HOME[1], 180.0), v500, fine)  # 제자리에서 갈 방향(-x)으로 돈다
        yield MoveBase(basetarget(PALLET_BUF[0], AMMR_HOME[1], 180.0), v500, fine)  # 버퍼 앞까지 직진한다
        yield MoveBase(bBuf, v500, fine)  # 버퍼 팔레트 쪽(+y)을 보고 선다
        yield MoveAbsJ(jReach, 45.0, fine)
        yield WaitTime(1.0)
        yield MoveAbsJ(jTuck, 45.0, fine)
        yield MoveBase(basetarget(PALLET_BUF[0], AMMR_HOME[1], 0.0), v500, fine)  # 제자리에서 갈 방향(+x)으로 돈다
        yield MoveBase(basetarget(AMMR_HOME[0], AMMR_HOME[1], 0.0), v500, fine)  # 적재 팔레트 앞까지 직진한다


def patrol_program():
    """H1 순찰 프로그램. 순찰 지점 네 곳을 차례로 걸어서 돈다.

    H1은 걷는 로봇이라 MoveBase의 뜻이 조금 다르다. 목표 쪽으로 몸을 돌려 걸어가고, 도착 방향 rz는 맞추지 않는다.
    z500이라 지점 0.5 m 안에 들어오면 멈추지 않고 다음 지점으로 향한다. 속도는 정책이 학습된 0 ~ 1 m/s 안에서 준다.
    """
    while True:
        for x, y in H1_WAYPOINTS:
            yield MoveBase(basetarget(x, y, 0.0), v500, z500)


def box_positions_rapid(t, tcp_pos):
    """RAPID 방식일 때 박스 6개의 위치. tcp_pos는 [m] 지금 실제 TCP 위치다.

    벨트 위를 흐르는 것은 시간표 방식과 같다. 집히고 놓이는 것만 시각이 아니라 doGrip 신호를 따른다.
    """
    out = box_positions(t, tcp_pos)  # 벨트 위 박스와, 시작할 때 이미 팔레트에 있던 박스는 이 값을 그대로 쓴다
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

    ik_step과 다른 점은 두 가지다. 손끝 링크가 아니라 tooldata로 정한 TCP를 맞추고,
    공구 축 방향만이 아니라 방향 전체 (rx, ry, rz)를 맞춘다.
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
    joint_step(arm, jac, error)
    if t > 3.0:  # 자체 점검용 기록 (초기 수렴 구간 제외)
        arm["errors"].append(np.linalg.norm(target - pos))
        arm["aligns"].append(float(axis @ DOWN))


def h1_pose():
    """H1 몸체의 지금 실제 자세를 ([m] x, [m] y, [rad] 바라보는 방향)으로 돌려준다."""
    pos, quat = h1.articulation.get_world_poses()
    (x, y, _), (qw, qx, qy, qz) = pos.numpy()[0], quat.numpy()[0]
    return x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def walk_command(pose, goal, speed=0.5):
    """H1을 [m] goal (x, y)로 보내는 [전진 속도, 횡 속도, 회전 속도] 명령. goal이 None이면 제자리에 선다.

    목표 방향과 몸 방향의 차이가 0.6 rad 미만이면 [m/s] speed로 걷고, 크면 제자리에서 돈다.
    """
    if goal is None:
        return np.zeros(3, dtype=np.float32)
    x, y, yaw = pose
    turn = (math.atan2(goal[1] - y, goal[0] - x) - yaw + math.pi) % (2 * math.pi) - math.pi
    return np.array([speed if abs(turn) < 0.6 else 0.0, 0.0, np.clip(1.5 * turn, -0.7, 0.7)], dtype=np.float32)


def patrol_goal(pose):
    """시간표 방식의 순찰. 지금 향하는 순찰 지점을 돌려주고, 0.5 m 안에 들어왔으면 다음 지점으로 바꿔 둔다."""
    gx, gy = H1_WAYPOINTS[state["waypoint"]]
    if math.hypot(gx - pose[0], gy - pose[1]) < 0.5:
        state["waypoint"] = (state["waypoint"] + 1) % len(H1_WAYPOINTS)
    return gx, gy


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

# ------------------------------------------------------------------ RAPID 방식의 목표·공구 등록
# RobotStudio에서는 물체 윗면에 스냅을 걸고 클릭해서 타깃을 찍는다. 여기서 같은 일을 하는 방법은 세 가지다.
#   1. snap_top("/World/물체경로")     코드에서 물체 윗면 가운데를 바로 robtarget으로 받는다. 아래 pPallet이 그 예다.
#   2. 실행 중에 물체를 클릭한다        터미널에 "[TEACH] robtarget(...)" 한 줄이 찍힌다. 그 줄을 복사해 여기에 붙인다.
#                                      찍힌 자리에는 빨간 표시가 생긴다. 아래 on_stage_event가 이 일을 한다.
#   3. 숫자를 직접 적는다               벨트 끝처럼 물체가 아니라 "라인에서 정한 위치"일 때 쓴다. 아래 pPick이 그 예다.
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


# tooldata. 손끝 링크에서 본 TCP의 [m] 위치 (x, y, z)와 [deg] 방향 (rx, ry, rz)이다. TCP의 Z축이 공구가 뻗는 방향이다.
# Kawasaki. 손끝 링크(gripper_center)의 +Z가 공구 방향이라 회전은 0이다. 그 방향으로 GAP만큼 나간 점을 TCP로 잡았다.
# 그래서 목표 좌표에 "박스 윗면"을 그대로 적으면 된다.
tGripper = tooldata(0.0, 0.0, GAP, 0.0, 0.0, 0.0)
# KUKA. 손끝 링크(tool0)는 +X가 공구 방향이다. y축으로 90도 돌려 TCP의 Z축을 그 방향에 맞춘다.
tScanner = tooldata(0.0, 0.0, 0.0, 0.0, 90.0, 0.0)

# robtarget. [m] 위치 (x, y, z)와 [deg] 방향 (rx, ry, rz)이다. rx = 180은 공구가 바닥을 보는 자세다.
pPallet = Offs(snap_top("/World/PalletOut"), 0.0, 0.0, BOX)  # 팔레트 윗면 가운데를 스냅으로 찍고, 박스 한 개 높이만큼 올린 점
pPick = robtarget(X_OUT, 0.0, BELT_Z + BOX, 180.0, 0.0, 0.0)  # 벨트 끝에 온 박스의 윗면. 박스가 서는 위치는 라인에서 정한 값이다
pScanStart = robtarget(-1.45, 0.0, BELT_Z + BOX + 0.10, 180.0, 0.0, 0.0)  # 검사를 시작하는 점. 박스 윗면에서 10 cm 위
pScanEnd = Offs(pScanStart, 0.90, 0.0, 0.0)  # 검사를 끝내는 점. 벨트 방향으로 0.9 m 간 곳

# jointtarget. [deg] AMMR 팔 7축의 각도다. 시간표 방식이 쓰는 TUCK, REACH([rad])와 같은 자세다.
jTuck = jointtarget(*np.degrees(TUCK))  # (0, -57.3, 0, -137.5, 0, 91.7, 45.0) 접은 자세
jReach = jointtarget(*np.degrees(REACH))  # (0, 43.0, 0, -74.5, 0, 117.5, 45.0) 앞으로 뻗은 자세

# basetarget. [m] 몸체 위치 (x, y)와 [deg] 바라보는 방향 rz다 (0이 +x, 90이 +y).
bOut = basetarget(AMMR_HOME[0], AMMR_HOME[1], 90.0)  # 적재 팔레트 앞. 팔레트를 본다
bBuf = basetarget(PALLET_BUF[0], AMMR_HOME[1], 90.0)  # 출하 버퍼 팔레트 앞. 버퍼를 본다

# ------------------------------------------------------------------ 실행
# 실행 중에 바뀌는 값들. t는 [s] 시뮬레이션 시각, first는 "재생 후 첫 스텝인가", waypoint는 H1이 향하는
# 순찰 지점 번호, error는 물리 콜백에서 난 오류 내용이다.
# tasks는 RAPID 방식일 때 로봇마다의 프로그램을 실행하는 RapidTask들이다. ammr_x는 방금 AMMR에 보낸 [m] 베이스 x 목표,
# boxes는 방금 계산한 박스 6개의 [m] 좌표, taught는 마지막으로 클릭해서 찍은 (물체 경로, [m] 좌표)다.
state = {"t": START_T, "first": True, "waypoint": 0, "error": None, "tasks": None, "ammr_x": 0.0, "boxes": None, "taught": None}

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
            # RAPID 방식의 프로그램·신호·박스 상태도 처음으로 되돌린다. 프로그램은 첫 줄부터 다시 시작한다.
            # 로봇마다 태스크가 하나씩이다. H1은 걷는 로봇이라 walk=True로 만든다.
            # scan_program()만 호출하면 제너레이터를 만들 뿐 로봇은 움직이지 않는다.
            # 매 스텝의 step()이 yield 명령을 소비하고, 그 직후 아래 제어 API들이 지령을 로봇에 적용한다.
            state["tasks"] = {
                "kuka": RapidTask(scan_program(), io),
                "kawasaki": RapidTask(palletize_program(), io),
                "ammr": RapidTask(ammr_program(), io),
                "h1": RapidTask(patrol_program(), io, walk=True),
            }
            io["doGrip"] = 0
            cell.update(held=None, placed={})
            for robot in (kuka["robot"], kawasaki["robot"], ammr):
                robot.reset_to_default_state()  # set_default_state로 정해 둔 자세로 한 번에 옮긴다
            # H1을 기본 자세로 세우고 보행 정책을 초기화한다. 인자는 속도 명령 [전진, 횡, 회전]이고 처음에는 0이다.
            h1.restart_from_default_state(np.zeros(3, dtype=np.float32))
            return
        # --- 매 스텝. 시계를 dt만큼 진행시키고, 그 시각에 맞는 목표를 로봇마다 보낸다.
        state["t"] += dt
        t = state["t"]
        pose = h1_pose()  # H1의 지금 실제 자세 ([m] x, [m] y, [rad] 방향)
        if args.motion == "rapid":
            # --- RAPID 방식. 로봇마다 프로그램을 dt만큼 진행시키고, 프로그램이 내놓은 "지금 있어야 할 상태"로 보낸다.
            tasks = state["tasks"]
            # 팔 두 대. 프로그램이 지령 TCP 자세를 내놓고, IK가 TCP를 그 자세로 보낸다.
            for arm, name in ((kuka, "kuka"), (kawasaki, "kawasaki")):
                link_pos, link_rot = link_pose(arm)  # 손끝 링크의 지금 실제 자세
                # 1. 피드백을 실행기에 전달한다. fine의 도착 판정은 지령이 아니라 이 실측값을 사용한다.
                tasks[name].step(dt, tcp=(link_pos, link_rot))
                # 2. 실행기가 만든 지령 TCP를 IK에 넘긴다. joint_step의 관절 목표 전송까지 여기서 이어진다.
                ik_step_pose(arm, tasks[name].tool, tasks[name].pos, tasks[name].rot, link_pos, link_rot, t)
            # AMMR. 프로그램이 지령 몸체 자세(월드 좌표)와 지령 팔 관절각을 내놓는다.
            # 베이스 관절값은 처음 놓은 자리(AMMR_HOME)가 0이므로 월드 좌표에서 그만큼 빼고 더한다.
            q = ammr.get_dof_positions().numpy()[0]
            tasks["ammr"].step(dt, base=(q[0] + AMMR_HOME[0], q[1] + AMMR_HOME[1], q[2]), joints=q[3:10])
            base, arm_joints = tasks["ammr"].base, tasks["ammr"].joints
            # MoveBase는 차체용 가상 관절 3개, MoveAbsJ는 Panda 팔 7개에 대응한다. 바퀴 속도 제어는 아니다.
            ammr_targets = [base[0] - AMMR_HOME[0], base[1] - AMMR_HOME[1], base[2], *arm_joints]
            # H1. 프로그램은 "어디로 얼마나 빨리 걸어가라"는 목표만 내놓는다. 걷는 것은 보행 정책이 한다.
            tasks["h1"].step(dt, base=pose)
            goal = tasks["h1"].goal
            # MoveBase 목표를 몸체 속도 [전진 m/s, 횡 m/s, 회전 rad/s]로 바꿔 아래 h1.step에 전달한다.
            command = walk_command(pose, None) if goal is None else walk_command(pose, goal[0], goal[1])
        else:
            # --- 시간표 방식. 지금 시각 t에 해당하는 목표를 시간표에서 꺼내 보낸다.
            ik_step(kuka, scan_target(t), t)  # KUKA 손끝을 "지나가는 박스 위" 좌표로 보낸다
            ik_step(kawasaki, palletize_target(t), t)  # Kawasaki 손끝을 "집기 → 놓기" 경로 위의 좌표로 보낸다
            ammr_targets = path(AMMR_KEYS, t)  # AMMR은 이 시각의 키프레임 값
            command = walk_command(pose, patrol_goal(pose))  # H1은 다음 순찰 지점 쪽으로
        # AMMR 관절 10개(베이스 3 + 팔 7)에 목표를 보낸다. 손가락 2개는 건드리지 않는다.
        # RAPID/시간표 분기는 '목표를 만드는 방법'만 고른다. 어느 쪽도 이 실제 관절 구동 단계를 생략하지 않는다.
        ammr.set_dof_position_targets(ammr_targets, dof_indices=list(range(10)))
        state["ammr_x"] = ammr_targets[0]
        h1.step(dt, command)  # H1 정책에 속도 명령을 넘긴다. 관절 19개의 목표는 정책이 계산한다
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
    if args.motion == "rapid":  # RAPID 방식은 tooldata로 정한 TCP 위치를 넘긴다
        state["boxes"] = box_positions_rapid(state["t"], link_pos + link_rot.apply(tGripper["pos"]))
    else:  # 시간표 방식은 그리퍼 링크 위치를 넘긴다
        state["boxes"] = box_positions(state["t"], link_pos)
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
    if args.motion == "rapid":
        # KUKA가 박스 위를 나란히 가는 구간에서, TCP와 가장 가까운 벨트 위 박스의 x 차이를 잰다.
        scan_pos, scan_rot = link_pose(kuka)
        scan_tcp = scan_pos + scan_rot.apply(tScanner["pos"])
        low_enough = scan_tcp[2] < pScanStart["pos"][2] + 0.02
        if low_enough and pScanStart["pos"][0] + 0.05 < scan_tcp[0] < pScanEnd["pos"][0] - 0.05:
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
    if args.motion == "rapid":
        # RAPID 방식 전용 항목. 프로그램이 박스를 제때 집어 팔레트 칸에 정확히 놓았는지 본다.
        slots = [np.array([PALLET_OUT[0] + dx, PALLET_OUT[1], PALLET_TOP + BOX / 2]) for dx in SLOT_X]
        offs = [min(float(np.linalg.norm(placed - slot)) for slot in slots) for placed in cell["log"]]  # [m] 칸 중심과의 거리
        expected = int(max(state["t"] - 5.0, 0.0) // TAKT) + 1  # 박스는 TAKT마다 오고, 집은 뒤 5초 안에 놓여야 한다
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
simulation_app.close()  # Isaac Sim 프로그램 자체를 끝낸다
