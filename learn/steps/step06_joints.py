# (6단계) 물리 콜백에서 KUKA 관절 목표를 매 스텝 보내 관절을 움직이고, 지령과 실제 각도를 기록하는 스크립트
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
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from my_scene import KUKA_HOME, build_scene
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

scene = build_scene(simulation_app)
kuka = scene["kuka"]
state = {"t": 0.0, "first": True}


def on_physics_step(dt, context):
    """물리가 한 스텝(0.005 s) 진행될 때마다 Isaac Sim이 불러 준다. 로봇 제어는 여기서 한다."""
    if state["first"]:  # 재생 직후 한 번. 시작 자세와 관절 구동기 게인을 정한다
        for name in ("kuka", "kawasaki", "ammr"):
            scene[name].reset_to_default_state()
        kuka.set_dof_gains(5.73e8, 5.73e6)  # 앞이 강성 kp, 뒤가 감쇠 kd
        state["first"] = False
        return
    state["t"] += dt
    t = state["t"]
    target = np.array(KUKA_HOME, dtype=float)
    target[0] = 0.6 * math.sin(2 * math.pi * t / 4.0)  # [rad] a1을 ±0.6 rad, 4초 주기로 흔든다
    # 핵심 API. 관절 "목표 각도"를 보낸다. 순간이동이 아니라 관절 구동기(PD)가 게인에 따라 따라간다
    kuka.set_dof_position_targets(target, dof_indices=list(range(6)))
    if round(t / dt) % 10 == 0:  # 0.05 s마다 기록
        q = kuka.get_dof_positions().numpy()[0]  # [rad] 지금 실제 관절각
        print(f"J {t:.3f} {target[0]:.4f} {q[0]:.4f}", flush=True)


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
print("STEP06 OK", flush=True)
timeline.stop()
SimulationManager.deregister_callback(callback_id)
simulation_app.close()
