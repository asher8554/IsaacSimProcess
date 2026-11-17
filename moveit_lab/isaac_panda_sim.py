# MoveIt이 계획한 궤적을 받아 Franka Panda를 움직이는 Isaac Sim 6.2 쪽 노드 (/isaac_panda)
r"""실행: C:\isaacsim-6.2\python.bat moveit_lab\isaac_panda_sim.py [--headless] [--seconds 30]

이 파일은 MoveIt 실습에서 "로봇 몸체 + 로봇 드라이버" 자리를 맡는다. 경로 계획은 하지 않는다.
경로 계획은 WSL Ubuntu에서 도는 MoveIt(move_group)이 하고, 이 파일은 받은 궤적을 그대로 따라간다.

    [WSL]  예제 스크립트 ──/move_action──▶ move_group (IK·OMPL·충돌 검사·시간 붙이기)
                                              │  FollowJointTrajectory 액션
                                              ▼  /panda_arm_controller/follow_joint_trajectory
    [Windows]                          /isaac_panda (이 파일) ──▶ Panda 관절 구동기 (PhysX)
                                              │
                                              └──▶ /joint_states, /clock ──▶ move_group, RViz

RAPID로 치면 move_group이 컨트롤러 안의 경로 계획기이고, 이 파일은 서보 드라이브와 모터다.
실제 로봇이라면 이 자리에 제조사 드라이버(ros2_control 하드웨어 플러그인)가 들어간다.

주고받는 것 (topic 이름은 MoveIt 기본 설정에 맞췄다. 바꾸면 MoveIt 설정도 같이 바꿔야 한다).
    발행  /joint_states   sensor_msgs/JointState   관절 9개(팔 7 + 손가락 2)의 실제 각도·속도. 물리 스텝마다
    발행  /clock          rosgraph_msgs/Clock       시뮬레이션 시각. MoveIt이 use_sim_time으로 이 시계를 쓴다
    액션  /panda_arm_controller/follow_joint_trajectory   control_msgs/FollowJointTrajectory
          MoveIt이 시간이 붙은 관절 궤적을 보낸다. 이 파일이 받아서 물리 스텝마다 보간해 관절 목표로 넣는다

MoveIt 없이 이 액션만 시험하려면 다른 터미널에서 send_trajectory.py를 실행한다.
"""

import argparse
import math
import os
import sys
import time
import traceback

# ROS 2 통신 구현을 FastDDS로 정한다. WSL의 ROS 2 Jazzy 기본값도 FastDDS라서 따로 맞출 것이 없다.
# Isaac Sim의 python.bat 기본값(Zenoh)은 라우터가 필요하므로 쓰지 않는다. SimulationApp보다 먼저 정해야 한다.
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
# 작은 행렬 계산뿐이라 수치 라이브러리 스레드를 1개로 묶는다. 여러 개면 물리 스레드와 CPU를 다투기만 한다.
for _name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_name, "1")

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="창 없이 실행")
parser.add_argument(
    "--seconds", type=float, default=0.0, help="이 시간(시뮬레이션 초)만 돌리고 자체 점검 후 종료. 0이면 창을 닫을 때까지 실행"
)
args = parser.parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.timeline
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim
from isaacsim.core.rendering_manager import RenderingManager, ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.storage.native import get_assets_root_path

# ROS 2 브리지 확장을 켜야 Isaac Sim에 들어 있는 ROS 2 Jazzy 라이브러리(rclpy, control_msgs 등)를 import할 수 있다.
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lab_common import (  # noqa: E402
    ARM_JOINTS,
    FINGER_JOINTS,
    FINGER_OPEN,
    HAND_LINK,
    OBSTACLE,
    P_LEFT,
    P_RIGHT,
    READY,
    TRAJECTORY_ACTION,
)

# ------------------------------------------------------------------ 이 파일에서만 쓰는 상수
PHYSICS_DT = 0.005  # [s] 물리 한 스텝. 200 Hz. 실제 로봇 드라이버의 제어 주기에 해당한다
GOAL_TOL = 0.02  # [rad] 궤적이 끝났을 때 관절마다 허용하는 오차. 넘으면 아직 도착 못 한 것으로 본다
GOAL_TIME_TOL = 2.0  # [s] 궤적 끝 시각이 지나고도 이만큼 기다려 준다. 그래도 못 오면 실패(abort)로 돌려준다
FEEDBACK_DT = 0.05  # [s] MoveIt에 진행 상황(feedback)을 보내는 간격


def ros_time(seconds):
    """[s] 실수 시각을 ROS 메시지 시각(초 + 나노초)으로 바꾼다."""
    sec = math.floor(seconds)
    return Time(sec=int(sec), nanosec=int((seconds - sec) * 1e9))


def duration(d):
    """궤적 점의 time_from_start(초 + 나노초)를 [s] 실수로 바꾼다."""
    return d.sec + d.nanosec * 1e-9


# ------------------------------------------------------------------ 궤적 보간
class Trajectory:
    """MoveIt이 보낸 관절 궤적. 물리 스텝마다 sample(t)로 그 시각의 관절 목표를 꺼낸다.

    MoveIt 궤적 점은 드문드문하다(보통 수십 개). 물리 스텝(5 ms)마다 두 점 사이를 보간해야 부드럽게 움직인다.
    점마다 속도가 있으면 Hermite 3차 곡선으로, 없으면 직선으로 잇는다.
    RAPID 컨트롤러가 내부에서 하는 보간과 같은 일이다. 다만 여기서는 MoveIt이 이미 속도·가속 한계를 맞춰 둔 점을 잇기만 한다.
    """

    def __init__(self, msg, q_now):
        # 궤적의 관절 순서를 이 파일의 ARM_JOINTS 순서로 바꾼다. MoveIt이 다른 순서로 보낼 수도 있다.
        order = [msg.joint_names.index(name) for name in ARM_JOINTS]
        self.times = np.array([duration(p.time_from_start) for p in msg.points])  # [s] 궤적 시작부터 각 점까지
        self.pos = np.array([[p.positions[i] for i in order] for p in msg.points])  # [rad] 점마다 관절 7개
        has_vel = all(len(p.velocities) == len(msg.joint_names) for p in msg.points)
        self.vel = np.array([[p.velocities[i] for i in order] for p in msg.points]) if has_vel else None  # [rad/s]
        if self.times[0] > 0.0:  # 첫 점이 0초가 아니면 지금 자세를 0초 점으로 앞에 붙인다
            self.times = np.insert(self.times, 0, 0.0)
            self.pos = np.vstack([q_now, self.pos])
            if self.vel is not None:
                self.vel = np.vstack([np.zeros(7), self.vel])
        self.end = float(self.times[-1])  # [s] 궤적 길이

    def sample(self, t):
        """궤적 시작 후 [s] t 시각의 관절 목표 [rad]. 범위 밖이면 첫 점·마지막 점을 유지한다."""
        if t <= self.times[0]:
            return self.pos[0].copy()
        if t >= self.end:
            return self.pos[-1].copy()
        i = int(np.searchsorted(self.times, t, side="right")) - 1  # t가 들어 있는 구간 번호
        dt = self.times[i + 1] - self.times[i]
        u = (t - self.times[i]) / dt  # [단위 없음] 구간 안에서의 위치 0~1
        a, b = self.pos[i], self.pos[i + 1]
        if self.vel is None:
            return a + (b - a) * u
        va, vb = self.vel[i], self.vel[i + 1]
        return (
            (2 * u**3 - 3 * u**2 + 1) * a
            + (u**3 - 2 * u**2 + u) * dt * va
            + (-2 * u**3 + 3 * u**2) * b
            + (u**3 - u**2) * dt * vb
        )


# ------------------------------------------------------------------ 장면
# 빈 무대에 바닥·조명·Panda·기둥 하나를 놓는다. 창고 같은 큰 환경은 쓰지 않는다 (첫 실행 다운로드가 짧다).
assets = get_assets_root_path()
stage_utils.create_new_stage()
GroundPlane("/World/GroundPlane", positions=[0.0, 0.0, 0.0])  # 충돌체가 있는 바닥. z = 0
DistantLight("/World/Light").set_intensities(1500)

# Panda. 로봇 베이스(panda_link0)를 월드 원점에 놓는다. MoveIt 쪽도 world → panda_link0을 원점으로 둔다(launch 파일).
# 내 로봇으로 바꿀 때는 이 경로를 내 로봇 USD로 바꾸고, lab_common.ARM_JOINTS를 그 로봇의 관절 이름으로 바꾼다.
# USD의 관절 이름과 MoveIt URDF의 관절 이름이 다르면 MoveIt 궤적을 받을 수 없다(아래 on_goal에서 거부).
stage_utils.add_reference_to_stage(assets + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd", "/World/Panda")
panda = Articulation("/World/Panda")
hand = RigidPrim(f"/World/Panda/{HAND_LINK}")  # 점검 때 손 링크의 실제 위치를 읽는 데 쓴다

# 기둥. MoveIt 예제 3이 Planning Scene에 같은 크기·위치로 등록한다. 여기 있는 것은 "진짜 물체"이고,
# Planning Scene에 등록하는 것은 "MoveIt이 아는 물체"다. 등록하지 않으면 MoveIt은 기둥을 모른 채 뚫고 지나가는 경로를 만들고,
# 이 시뮬레이터에서 팔이 실제로 기둥에 부딪힌다. 충돌체(GeomPrim)는 있고 강체는 없어서 움직이지 않는다.
pillar = Cube("/World/Pillar", sizes=1.0, scales=[list(OBSTACLE["size"])], positions=[list(OBSTACLE["center"])], colors=[0.8, 0.25, 0.2])
GeomPrim("/World/Pillar", apply_collision_apis=True)
# 목표점 표시. 예제 3의 두 목표 TCP 위치에 작은 초록 상자를 둔다. 충돌체가 없는 표시용이다.
Cube(["/World/Marks/left", "/World/Marks/right"], sizes=0.03, positions=[list(P_LEFT), list(P_RIGHT)], colors=[0.2, 0.8, 0.3])

SimulationManager.set_physics_dt(PHYSICS_DT)
RenderingManager.set_dt(8 * PHYSICS_DT)  # [s] 화면은 물리 8스텝마다 한 번 그린다 (25 Hz)
while stage_utils.is_stage_loading():  # 에셋 서버에서 Panda 파일을 다 받을 때까지 기다린다
    simulation_app.update()
ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.8, -1.6, 1.3], target=[0.3, 0.0, 0.35])

# 관절 이름으로 관절 번호를 찾는다. USD 안의 관절 순서는 MoveIt과 다를 수 있으므로 번호를 직접 적지 않는다.
dof_names = list(panda.dof_names)
missing = [n for n in ARM_JOINTS + FINGER_JOINTS if n not in dof_names]
if missing:
    raise SystemExit(f"Panda USD에 없는 관절 이름: {missing}. 가진 관절: {dof_names}")
ARM_IDX = [dof_names.index(n) for n in ARM_JOINTS]
JS_NAMES = ARM_JOINTS + FINGER_JOINTS  # /joint_states에 싣는 순서
JS_IDX = [dof_names.index(n) for n in JS_NAMES]
home = np.zeros(len(dof_names))
home[ARM_IDX] = READY
home[[dof_names.index(n) for n in FINGER_JOINTS]] = FINGER_OPEN
panda.set_default_state(dof_positions=home.tolist())  # 재생할 때마다 ready 자세에서 시작한다
_kp, _kd = (g.numpy()[0][ARM_IDX] for g in panda.get_dof_gains())
print(f"[MOVEIT-LAB] arm drive gains kp {np.round(_kp, 1).tolist()} kd {np.round(_kd, 1).tolist()}", flush=True)

# ------------------------------------------------------------------ ROS 2 노드
# 노드 하나에 발행자 2개와 액션 서버 1개가 있다. 콜백은 물리 콜백 안에서 executor.spin_once()로 돌린다.
# 그래서 모든 콜백이 물리와 같은 스레드에서 돌고, 관절 값을 읽고 쓸 때 잠금이 필요 없다.
rclpy.init()
ros = rclpy.create_node("isaac_panda")
pub_js = ros.create_publisher(JointState, "/joint_states", 10)
pub_clock = ros.create_publisher(Clock, "/clock", 10)

# 액션 진행 상태. 한 번에 궤적 하나만 실행한다(실제 컨트롤러와 같다).
# pending  수락했지만 아직 시작 시각이 안 정해진 목표 (다음 물리 스텝에서 시작한다)
# active   실행 중인 목표. handle, 궤적, 시작 시각, 끝났을 때 결과를 넣을 future가 있다
run = {"pending": None, "active": None, "log": []}
state = {"t": 0.0, "first": True, "error": None, "q_hold": np.array(READY), "js_count": 0, "last_fb": 0.0}


def on_goal(goal):
    """MoveIt이 궤적을 보내면 먼저 불린다. 받을지 말지만 정한다 (RAPID에서 경로 실행 전에 하는 검사).

    거부하는 경우
      - 이미 다른 궤적을 실행 중이다
      - 관절 이름이 ARM_JOINTS와 다르다 (다른 로봇·다른 그룹 설정이 보낸 궤적)
      - 점이 하나도 없다
    """
    traj = goal.trajectory
    if run["active"] or run["pending"]:
        ros.get_logger().warning("reject: 이미 다른 궤적을 실행 중")
        return GoalResponse.REJECT
    if sorted(traj.joint_names) != sorted(ARM_JOINTS) or not traj.points:
        ros.get_logger().warning(f"reject: 관절 이름이 다르거나 점이 없음 {list(traj.joint_names)}")
        return GoalResponse.REJECT
    return GoalResponse.ACCEPT


def on_cancel(_goal_handle):
    """RViz의 Stop이나 MoveIt 실행 중단 요청. 받아 주고, 실제 정지는 물리 콜백이 처리한다."""
    return CancelResponse.ACCEPT


async def on_execute(goal_handle):
    """수락한 목표를 실행한다. 여기서는 물리 콜백에 넘기고 결과가 나올 때까지 기다리기만 한다.

    async 함수라서 기다리는 동안 다른 콜백을 막지 않는다. 물리 콜백이 future에 결과를 넣으면
    다음 spin_once 때 이 함수가 이어서 돌고 결과를 MoveIt에 돌려준다.
    """
    done = rclpy.task.Future()
    run["pending"] = {"handle": goal_handle, "future": done}
    return await done


ActionServer(
    ros,
    FollowJointTrajectory,
    TRAJECTORY_ACTION,
    execute_callback=on_execute,
    goal_callback=on_goal,
    cancel_callback=on_cancel,
)
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(ros)


def finish(code, text, how):
    """실행 중인 목표를 끝낸다. how는 'succeed'(성공), 'abort'(실패), 'canceled'(취소) 중 하나다."""
    active = run["active"]
    result = FollowJointTrajectory.Result(error_code=code, error_string=text)
    getattr(active["handle"], how)()  # 액션 상태를 바꾼다. MoveIt은 이 상태로 성공·실패를 판단한다
    active["future"].set_result(result)  # on_execute가 기다리던 결과를 넣는다
    run["log"].append((how, text))
    ros.get_logger().info(f"trajectory {how}: {text}")
    run["active"] = None


def follow(t, q):
    """실행 중인 궤적을 한 스텝 진행한다. t는 [s] 지금 시각, q는 [rad] 팔 관절 7개의 실제 값이다."""
    if run["pending"]:  # 새 목표를 지금 시각에 시작한다
        p, run["pending"] = run["pending"], None
        msg = p["handle"].request.trajectory
        run["active"] = {**p, "traj": Trajectory(msg, q), "start": t}
        state["last_fb"] = -math.inf
        ros.get_logger().info(f"trajectory start: {len(msg.points)} points, {run['active']['traj'].end:.2f} s")
    active = run["active"]
    if not active:
        return
    if active["handle"].is_cancel_requested:  # 취소면 지금 자세에서 멈춘다
        state["q_hold"] = q.copy()
        finish(FollowJointTrajectory.Result.SUCCESSFUL, "canceled", "canceled")
        return
    traj, s = active["traj"], t - active["start"]  # [s] s는 궤적 시작 후 흐른 시간
    desired = traj.sample(s)
    state["q_hold"] = desired  # 끝난 뒤에도 마지막 목표를 계속 유지한다
    if s - state["last_fb"] >= FEEDBACK_DT or s < state["last_fb"]:  # 진행 상황을 MoveIt에 알린다
        state["last_fb"] = s
        fb = FollowJointTrajectory.Feedback(joint_names=ARM_JOINTS)
        fb.header.stamp = ros_time(t)
        fb.desired = JointTrajectoryPoint(positions=desired.tolist())
        fb.actual = JointTrajectoryPoint(positions=q.tolist())
        fb.error = JointTrajectoryPoint(positions=(desired - q).tolist())
        active["handle"].publish_feedback(fb)
    err = float(np.max(np.abs(traj.pos[-1] - q)))  # [rad] 마지막 점과 실제 관절의 최대 차이
    if s >= traj.end and err < GOAL_TOL:
        finish(FollowJointTrajectory.Result.SUCCESSFUL, f"reached (max error {err:.4f} rad)", "succeed")
    elif s > traj.end + GOAL_TIME_TOL:  # 부딪혀 멈췄거나 너무 늦다
        finish(FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, f"did not reach goal (max error {err:.3f} rad)", "abort")


def on_physics_step(dt, _context):
    """물리 한 스텝(5 ms)마다 불린다. 상태 발행 → 도착한 요청 처리 → 궤적 한 스텝 진행 순서다."""
    if state["error"]:
        return
    try:
        if state["first"]:  # 재생 직후 한 번. 정지 후 다시 재생했을 때도 여기로 온다
            state.update(t=0.0, first=False, q_hold=np.array(READY), last_fb=0.0)
            if run["active"]:  # 정지 때문에 끊긴 궤적은 실패로 돌려준다
                finish(FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, "simulation restarted", "abort")
            panda.reset_to_default_state()
            return
        state["t"] += dt
        t = state["t"]
        # 1. 내보내기. 관절 9개의 실제 값과 시계. MoveIt은 이 /joint_states를 "로봇의 현재 자세"로 쓴다.
        q_all = panda.get_dof_positions().numpy()[0]
        v_all = panda.get_dof_velocities().numpy()[0]
        js = JointState(name=JS_NAMES, position=[float(q_all[i]) for i in JS_IDX], velocity=[float(v_all[i]) for i in JS_IDX])
        js.header.stamp = ros_time(t)
        pub_js.publish(js)
        pub_clock.publish(Clock(clock=ros_time(t)))
        state["js_count"] += 1
        # 2. 받기. 도착한 액션 요청·취소를 처리한다. 기다리지 않는다(timeout 0).
        for _ in range(10):
            executor.spin_once(timeout_sec=0.0)
        # 3. 적용. 궤적을 진행하고 팔 관절 목표를 넣는다. 궤적이 없으면 마지막 목표를 유지한다.
        follow(t, q_all[ARM_IDX])
        panda.set_dof_position_targets(state["q_hold"].tolist(), dof_indices=ARM_IDX)
    except Exception:  # noqa: BLE001 - 물리 콜백은 예외를 밖으로 던지면 안 된다
        state["error"] = traceback.format_exc()
        carb.log_error(f"isaac_panda_sim: 물리 콜백 실패\n{state['error']}")


callback_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)
timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()
print(f"[MOVEIT-LAB] /isaac_panda ready | action {TRAJECTORY_ACTION} | publishing /joint_states, /clock", flush=True)

# ------------------------------------------------------------------ 메인 루프
wall_start = time.time()
while simulation_app.is_running() and not state["error"]:
    simulation_app.update()
    if timeline.is_stopped():  # GUI에서 정지하면 다음 재생 때 ready 자세부터 다시 시작한다
        state["first"] = True
    if args.seconds and state["t"] >= args.seconds:
        break

# ------------------------------------------------------------------ 자체 점검 (--seconds를 줬을 때만)
if args.seconds:
    hand_pos = hand.get_world_poses()[0].numpy()[0]
    q = panda.get_dof_positions().numpy()[0][ARM_IDX]
    print(f"INFO | hand link at {np.round(hand_pos, 3).tolist()} m, arm joints {np.round(q, 3).tolist()} rad")
    print(f"INFO | trajectories: {run['log'] or 'none received'}")
    checks = [
        ("Physics callback ran without error", state["error"] is None, state["error"] or "ok"),
        ("/joint_states published every step", state["js_count"] > 0.9 * args.seconds / PHYSICS_DT, f"{state['js_count']} messages"),
        ("Every received trajectory succeeded", all(how == "succeed" for how, _ in run["log"]), f"{len(run['log'])} received"),
    ]
    for name, ok, detail in checks:
        print(f"CHECK {'PASS' if ok else 'FAIL'} | {name} | {detail}")
    print(f"REALTIME {state['t'] / (time.time() - wall_start):.2f}x")
    print("SELF-CHECK", "PASS" if all(ok for _, ok, _ in checks) else "FAIL")

# ------------------------------------------------------------------ 종료
timeline.stop()
SimulationManager.deregister_callback(callback_id)
ros.destroy_node()
rclpy.shutdown()
simulation_app.close()
