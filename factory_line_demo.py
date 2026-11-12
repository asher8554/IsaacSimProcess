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

# ------------------------------------------------------------------ 라인 배치 (단위 m, s, rad)
TAKT = 8.0  # 박스 1개가 나오는 간격
BELT_V = 0.225  # 컨베이어 속도
BELT_Z = 0.77  # 롤러 윗면 높이 (ConveyorBelt_A05 실측)
BOX = 0.35  # 박스 한 변
X_IN, X_OUT = -3.7, 1.7  # 박스가 벨트에 올라오는 곳과 집히는 곳. 거리 = 3 * TAKT * BELT_V
GAP = 0.02  # 그리퍼 끝과 박스 윗면 사이
RELEASE = 4.6  # 택트 안에서 박스를 놓는 시각
PALLET_TOP = 0.143  # pallet.usd 실측 높이
PALLET_OUT = (3.2, -1.5)  # 로봇 B가 쌓는 팔레트
PALLET_BUF = (-3.0, -1.5)  # AMMR이 다녀오는 출하 버퍼 팔레트
SLOT_X = (-0.4, 0.0, 0.4)  # 팔레트 위 박스 3칸
START_T = -2.0  # 첫 박스가 집히기 2초 전 상태에서 시작한다
DOWN = np.array([0.0, 0.0, -1.0])

KUKA_HOME = [0.0, 0.0, 0.3, 0.0, 1.27, 0.0]  # 공구가 아래를 보는 자세
KAWASAKI_HOME = [0.0, 0.2, -1.5, 0.0, -1.44, 0.0] + [0.0] * 6  # 뒤 6개는 그리퍼
TUCK = [0.0, -1.0, 0.0, -2.4, 0.0, 1.6, 0.785]  # AMMR 팔 접은 자세
REACH = [0.0, 0.75, 0.0, -1.3, 0.0, 2.05, 0.785]  # AMMR 팔 앞으로 뻗은 자세
H = math.pi / 2
D = PALLET_BUF[0] - PALLET_OUT[0]  # 버퍼 정차 위치 (출발점 기준 x)
AMMR_KEYS = [  # (시각, [베이스 x, y, yaw, 팔 7축]). 마지막 시각이 주기
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
H1_WAYPOINTS = [(4.0, 4.4), (4.0, 6.0), (-4.0, 6.0), (-4.0, 4.4)]  # 순찰 경로 (반시계)


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
assets = get_assets_root_path()
if assets is None:
    raise RuntimeError("Isaac Sim 에셋 서버에 연결할 수 없다. 인터넷 연결을 확인한다.")

stage_utils.create_new_stage()
stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
SimulationManager.set_physics_sim_device("cpu")
simulation_app.update()


def add(url, prim_path, position=(0.0, 0.0, 0.0), yaw=0.0):
    """USD 에셋을 지정한 위치·방향으로 스테이지에 놓는다."""
    stage_utils.add_reference_to_stage(assets + url, prim_path)
    XformPrim(
        prim_path,
        positions=list(position),
        orientations=[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)],
        reset_xform_op_properties=True,
    )


add("/Isaac/Environments/Simple_Warehouse/warehouse.usd", "/World/Warehouse")
for i in range(3):  # 2 m 컨베이어 3개를 이어 x = -4 ~ 2 구간을 만든다
    add("/Isaac/Props/Conveyors/ConveyorBelt_A05.usd", f"/World/Conveyor_{i}", (-4.0 + 2.0 * i, 0.0, 0.0))
add("/Isaac/Props/Pallet/pallet.usd", "/World/PalletOut", (*PALLET_OUT, 0.0))
add("/Isaac/Props/Pallet/pallet.usd", "/World/PalletBuffer", (*PALLET_BUF, 0.0))
boxes = Cube([f"/World/Boxes/box_{i}" for i in range(6)], sizes=BOX, colors=[0.72, 0.52, 0.30])
Cube(
    [f"/World/BufferBoxes/box_{i}" for i in range(2)],
    sizes=BOX,
    colors=[0.72, 0.52, 0.30],
    positions=[[PALLET_BUF[0] + dx, PALLET_BUF[1], PALLET_TOP + BOX / 2] for dx in (-0.4, 0.0)],
)

add("/Isaac/Robots/Kuka/KR210_L150/kr210_l150.usd", "/World/Kuka", (-1.0, 2.2, 0.0), yaw=-H)
add("/Isaac/Robots/Kawasaki/RS080N/rs080n_onrobot_rg2.usd", "/World/Kawasaki", (3.2, 0.0, 0.0), yaw=-1.5 * H)
add("/Isaac/Robots/Clearpath/RidgebackFranka/ridgeback_franka.usd", "/World/AMMR", (PALLET_OUT[0], -3.0, 0.0))


def make_arm(prim_path, ee_link, tool_axis, home):
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


kuka = make_arm("/World/Kuka", "tool0", [1, 0, 0], KUKA_HOME)
kawasaki = make_arm("/World/Kawasaki", "gripper_center", [0, 0, 1], KAWASAKI_HOME)
ammr = Articulation("/World/AMMR")
ammr.set_default_state(dof_positions=[0.0, 0.0, 0.0, *TUCK, 0.04, 0.04])

h1 = RobotPolicyRunner(get_h1_spec(), prim_path="/World/H1", position=[-4.0, 4.4, 1.05])
h1.spawn()
SimulationManager.set_physics_dt(h1.physics_dt)  # 보행 정책이 학습된 물리 주기 (200 Hz)
RenderingManager.set_dt(8 * h1.physics_dt)  # 화면은 25 Hz

while stage_utils.is_stage_loading():
    simulation_app.update()
ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[8.5, -8.0, 6.0], target=[0.0, 0.5, 0.8])

# ------------------------------------------------------------------ 실행
state = {"t": START_T, "first": True, "waypoint": 0, "error": None}


def on_physics_step(dt, context):
    """물리 스텝마다 네 로봇의 목표를 갱신한다."""
    if state["error"]:
        return
    try:
        if state["first"]:
            state.update(t=START_T, first=False, waypoint=0)
            # KUKA 에셋 기본 게인(5.7e16)에서는 지령 없이 a1이 0.13 rad 흘렀다. 다른 팔과 같은 값으로 맞춘다.
            kuka["robot"].set_dof_gains(5.73e8, 5.73e6)
            for arm in (kuka, kawasaki):
                arm["q_cmd"] = arm["home"].copy()
            for robot in (kuka["robot"], kawasaki["robot"], ammr):
                robot.reset_to_default_state()
            h1.restart_from_default_state(np.zeros(3, dtype=np.float32))
            return
        state["t"] += dt
        t = state["t"]
        ik_step(kuka, scan_target(t), t)
        ik_step(kawasaki, palletize_target(t), t)
        ammr.set_dof_position_targets(path(AMMR_KEYS, t), dof_indices=list(range(10)))
        h1.step(dt, walk_command())
    except Exception:  # noqa: BLE001 - 물리 콜백은 예외를 밖으로 던지면 안 된다
        state["error"] = traceback.format_exc()
        carb.log_error(f"factory_line_demo: 물리 콜백 실패\n{state['error']}")


callback_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)
timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
shots = [6.0, 16.0, 26.0, 36.0] if args.seconds else []
h1_z_min, h1_travel, h1_last, ammr_err, ammr_x_min = math.inf, 0.0, None, 0.0, 0.0
wall_start = time.time()



def frame():
    """한 프레임을 돌린다. 시뮬레이션이 진행 중이면 현재 시각을, 아니면 None을 돌려준다."""
    simulation_app.update()
    if timeline.is_stopped():  # GUI에서 정지하면 다음 재생 때 처음부터 다시 시작한다
        if not state["first"]:
            # 정지하면 PhysX가 H1 관절 게인을 에셋 기본값(kp 5730)으로 되돌린다. 정책 런타임을 닫아 두면
            # 다음 재생 때 다시 만들어지면서 학습 게인(kp 150~200)이 재적용된다. 안 닫으면 H1이 넘어진다.
            h1.close()
            state["first"] = True
        return None
    if state["first"]:
        return None
    boxes.set_world_poses(positions=box_positions(state["t"], kawasaki["ee"].get_world_poses()[0].numpy()[0]))
    return state["t"]


while simulation_app.is_running() and not state["error"]:
    t = frame()
    if t is None or not args.seconds:
        continue
    # 아래는 자체 점검용 기록
    h1_pos = h1.articulation.get_world_poses()[0].numpy()[0]
    h1_z_min = min(h1_z_min, h1_pos[2])
    h1_travel += 0.0 if h1_last is None else float(np.linalg.norm(h1_pos[:2] - h1_last))
    h1_last = h1_pos[:2].copy()
    ammr_q = ammr.get_dof_positions().numpy()[0]
    ammr_err = max(ammr_err, abs(ammr_q[0] - path(AMMR_KEYS, t)[0]))
    ammr_x_min = min(ammr_x_min, ammr_q[0])
    if shots and t >= shots[0]:
        os.makedirs(out_dir, exist_ok=True)
        capture_viewport_to_file(get_active_viewport(), os.path.join(out_dir, f"shot_{int(shots.pop(0)):02d}s.png"))
    if t >= args.seconds:
        break

if args.seconds and state["error"]:
    print("SELF-CHECK FAIL | physics callback raised\n" + state["error"])
elif args.seconds:
    for _ in range(5):  # 마지막 캡처가 파일로 써지도록 몇 프레임 더 돈다
        simulation_app.update()
    elapsed = state["t"] - START_T
    checks = [
        ("KUKA mean tip error < 0.03 m", np.mean(kuka["errors"]) < 0.03, f"mean {np.mean(kuka['errors']):.4f} max {np.max(kuka['errors']):.4f}"),
        ("KUKA tool points down", np.mean(kuka["aligns"]) > 0.95, f"mean cos {np.mean(kuka['aligns']):.3f}"),
        ("Kawasaki mean tip error < 0.03 m", np.mean(kawasaki["errors"]) < 0.03, f"mean {np.mean(kawasaki['errors']):.4f} max {np.max(kawasaki['errors']):.4f}"),
        ("Kawasaki tool points down", np.mean(kawasaki["aligns"]) > 0.95, f"mean cos {np.mean(kawasaki['aligns']):.3f}"),
        ("AMMR base follows its path (< 0.3 m)", ammr_err < 0.3, f"max lag {ammr_err:.3f} m, reached x {ammr_x_min:.2f} m"),
        ("H1 stays upright (pelvis > 0.8 m)", h1_z_min > 0.8, f"min z {h1_z_min:.3f} m"),
        ("H1 keeps walking (> 0.25 m/s)", h1_travel / elapsed > 0.25, f"{h1_travel:.1f} m in {elapsed:.0f} s"),
    ]
    realtime = elapsed / (time.time() - wall_start)
    # 정지 후 재생 점검. 다시 재생한 뒤 10초 동안 H1 높이와 두 팔의 손끝 오차를 본다.
    seen = len(kuka["errors"]), len(kawasaki["errors"])
    timeline.stop()
    for _ in range(5):
        frame()
    timeline.play()
    replay_z = math.inf
    while simulation_app.is_running() and not state["error"]:
        t = frame()
        if t is None:
            continue
        replay_z = min(replay_z, h1.articulation.get_world_poses()[0].numpy()[0][2])
        if t >= 8.0:
            break
    replay_err = np.mean(kuka["errors"][seen[0] :] + kawasaki["errors"][seen[1] :])
    checks += [
        ("Replay ran without error", state["error"] is None, state["error"] or "ok"),
        ("H1 upright after stop and replay (pelvis > 0.8 m)", replay_z > 0.8, f"min z {replay_z:.3f} m"),
        ("Arms track after stop and replay (< 0.03 m)", replay_err < 0.03, f"mean {replay_err:.4f}"),
    ]
    for name, ok, detail in checks:
        print(f"CHECK {'PASS' if ok else 'FAIL'} | {name} | {detail}")
    print(f"REALTIME {realtime:.2f}x ({elapsed:.0f} sim s)")
    print("SELF-CHECK", "PASS" if all(ok for _, ok, _ in checks) else "FAIL")

timeline.stop()
SimulationManager.deregister_callback(callback_id)
h1.close()
simulation_app.close()
