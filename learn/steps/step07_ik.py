# (7단계) 미분 IK로 KUKA의 TCP(공구 끝)를 공중에 원을 그리게 움직이고 목표와의 오차를 기록하는 스크립트
import argparse
import math

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="창 없이 실행")
parser.add_argument("--frames", type=int, default=0, help="이 프레임 수만큼 돌고 끝낸다. 0이면 창을 닫을 때까지")
parser.add_argument("--shot", default="", help="끝나기 직전 화면을 저장할 PNG 경로")
args = parser.parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import numpy as np
import omni.timeline
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from my_scene import KUKA_HOME, build_scene
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from rapid_style import robtarget, tooldata  # 저장소의 rapid_style.py. numpy·scipy만 쓴다
from scipy.spatial.transform import Rotation

scene = build_scene(simulation_app)
robot = scene["kuka"]
ee = RigidPrim("/World/Kuka/tool0")  # 손끝 링크. 5단계에서 출력한 링크 이름 중 공구가 달린 것
row = robot.get_link_indices("tool0").list()[0] - 1  # 자코비안 행 번호. 고정 베이스라 루트 링크가 빠져 1을 뺀다
tool = tooldata(0.0, 0.0, 0.0, 0.0, 90.0, 0.0)  # tool0은 +X가 공구 방향이라 y축으로 90도 돌려 TCP의 Z를 맞춘다
DOWN = robtarget(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)["rot"]  # rx = 180: 공구가 바닥을 보는 방향
CENTER, RADIUS, PERIOD = np.array([-1.0, 0.7, 1.35]), 0.25, 6.0  # [m] 원 중심, [m] 반지름, [s] 한 바퀴
state = {"t": 0.0, "first": True, "q_cmd": np.array(KUKA_HOME, dtype=float)}


def link_pose():
    """손끝 링크의 지금 실제 ([m] 위치, 방향). Isaac Sim 쿼터니언은 (w,x,y,z), scipy는 (x,y,z,w) 순서다."""
    pos, quat = ee.get_world_poses()
    w, x, y, z = quat.numpy()[0]
    return pos.numpy()[0], Rotation.from_quat([x, y, z, w])


def ik_step(target_pos, target_rot):
    """미분 IK 한 스텝. TCP 오차 → 자코비안 → 관절 증분 → 관절 목표. 돌려주는 값은 [m] 위치 오차."""
    link_pos, link_rot = link_pose()
    reach = link_rot.apply(tool["pos"])  # [m] 손끝 링크 원점 → TCP (월드 기준)
    tcp_pos, tcp_rot = link_pos + reach, link_rot * tool["rot"]
    error = np.concatenate([target_pos - tcp_pos, (target_rot * tcp_rot.inv()).as_rotvec()])  # 위치 3 + 방향 3
    jac = robot.get_jacobian_matrices().numpy()[0, row, :, :6].copy()
    jac[:3] -= np.cross(reach, jac[3:].T).T  # 링크 원점 속도를 TCP 속도로 바꾼다 (v_tcp = v + w x reach)
    dq = jac.T @ np.linalg.solve(jac @ jac.T + 0.01 * np.eye(6), error)  # 감쇠 최소제곱
    q = robot.get_dof_positions().numpy()[0, :6]
    # 지령 관절각을 따로 적분한다. "실제 각도 + 증분"으로 보내면 PD 구동 때문에 느리게 끌려간다 (함정 1)
    state["q_cmd"] = np.clip(state["q_cmd"] + np.clip(0.3 * dq, -0.01, 0.01), q - 0.1, q + 0.1)
    robot.set_dof_position_targets(state["q_cmd"], dof_indices=list(range(6)))
    return float(np.linalg.norm(target_pos - tcp_pos))


def on_physics_step(dt, context):
    if state["first"]:
        robot.reset_to_default_state()
        robot.set_dof_gains(5.73e8, 5.73e6)
        state["first"] = False
        return
    state["t"] += dt
    t = state["t"]
    a = 2 * math.pi * t / PERIOD
    target = CENTER + RADIUS * np.array([math.cos(a), math.sin(a), 0.0])  # [m] 이번 스텝의 목표 TCP
    err = ik_step(target, DOWN)
    if round(t / dt) % 10 == 0:
        link_pos, link_rot = link_pose()
        actual = link_pos + link_rot.apply(tool["pos"])  # [m] 지금 실제 TCP
        print(f"E {t:.3f} {err:.4f} {target[0]:.4f} {target[1]:.4f} {actual[0]:.4f} {actual[1]:.4f}", flush=True)


callback_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)
timeline = omni.timeline.get_timeline_interface()
timeline.play()
frame = 0
while simulation_app.is_running():
    simulation_app.update()
    frame += 1
    if args.shot and frame == max(args.frames - 5, 1):
        capture_viewport_to_file(get_active_viewport(), args.shot)
    if args.frames and frame >= args.frames:
        break
print("STEP07 OK", flush=True)
timeline.stop()
SimulationManager.deregister_callback(callback_id)
simulation_app.close()
