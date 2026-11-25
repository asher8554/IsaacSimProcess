# (4단계) CAD에서 직접 리깅한 ABB GoFa USD를 공장 장면에 놓고 Articulation으로 잡아 관절을 움직이는 스크립트
import argparse
import math

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="창 없이 실행")
parser.add_argument("--frames", type=int, default=0, help="이 프레임 수만큼 돌고 끝낸다. 0이면 창을 닫을 때까지")
parser.add_argument("--shot", default="", help="끝나기 직전 화면을 저장할 PNG 경로")
parser.add_argument(
    "--usd",
    default="E:/Github/Isaac_Sim_tutorial/study/gofa_10kg_152/urdf/gofa_repaired_v2/gofa_physics_repaired.usda",
    help="내가 만든 로봇 USD 파일 경로",
)
args = parser.parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.timeline
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from my_scene import add, build_scene
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import UsdPhysics

build_scene(simulation_app, with_robots=False)
add(args.usd, "/World/GoFa", (-0.5, -1.8, 0.0), yaw=math.pi / 2)  # 내 PC의 파일도 같은 add()로 놓는다
while stage_utils.is_stage_loading():
    simulation_app.update()

# Articulation Root가 붙은 프림을 찾는다. 리깅할 때 어디에 붙였는지에 따라 경로가 다르다.
stage = stage_utils.get_current_stage()
roots = [str(p.GetPath()) for p in stage.Traverse()
         if str(p.GetPath()).startswith("/World/GoFa") and p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print("ROOT", roots, flush=True)
gofa = Articulation(roots[0])
ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.6, -4.4, 1.9], target=[-0.5, -1.8, 0.8])
state = {"t": 0.0, "first": True}


def on_physics_step(dt, context):
    if state["first"]:
        print(f"GOFA dof {gofa.num_dofs}: {list(gofa.dof_names)}", flush=True)
        state["first"] = False
        return
    state["t"] += dt
    t = state["t"]
    target = np.zeros(gofa.num_dofs)
    target[0] = 0.8 * math.sin(2 * math.pi * t / 6.0)  # [rad] joint_1(몸통 회전)을 ±0.8 rad, 6초 주기
    target[1] = 0.3 * math.sin(2 * math.pi * t / 6.0)  # [rad] joint_2(어깨)를 ±0.3 rad
    gofa.set_dof_position_targets(target)
    if round(t / dt) % 10 == 0:
        q = gofa.get_dof_positions().numpy()[0]
        print(f"G {t:.3f} {target[0]:.4f} {q[0]:.4f} {target[1]:.4f} {q[1]:.4f} {q[2]:.4f}", flush=True)


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
print("STEP04 OK", flush=True)
timeline.stop()
SimulationManager.deregister_callback(callback_id)
simulation_app.close()
