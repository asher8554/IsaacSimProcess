# (2단계) 빈 장면에 바닥과 상자를 만들고 물리를 돌려 상자가 떨어지는지 보는 최소 Isaac Sim 스크립트
import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="창 없이 실행")
parser.add_argument("--frames", type=int, default=0, help="이 프레임 수만큼 돌고 끝낸다. 0이면 창을 닫을 때까지")
parser.add_argument("--shot", default="", help="끝나기 직전 화면을 저장할 PNG 경로")
args = parser.parse_args()
# 여기서 Isaac Sim 앱이 켜진다. isaacsim·omni 모듈은 반드시 이 줄 뒤에서 import한다.
simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.timeline
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.rendering_manager import RenderingManager, ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

# --- 빈 장면과 물리. 길이 단위는 m, 위쪽은 +Z다.
stage_utils.create_new_stage()
stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")  # 중력 등 물리 설정을 담는 프림
SimulationManager.set_physics_sim_device("cpu")  # 로봇 몇 대 규모는 CPU 물리로 충분하다
SimulationManager.set_physics_dt(1 / 200)  # [s] 물리 한 스텝 0.005 s (200 Hz)
RenderingManager.set_dt(8 / 200)  # [s] 화면 한 장 간격 요청값. 실제로 몇 스텝 도는지는 아래 콜백으로 센다
simulation_app.update()

# --- 바닥과 상자. Cube는 모양만 만든다. 충돌체(GeomPrim)와 강체(RigidPrim)를 붙여야 물리가 적용된다.
ground = Cube("/World/Ground", sizes=1.0, positions=[[0.0, 0.0, -0.05]], scales=[[10.0, 10.0, 0.1]], colors=[0.35, 0.37, 0.4])
GeomPrim(ground.paths, apply_collision_apis=True)  # 바닥: 충돌체만. 강체가 아니라서 움직이지 않는다
box = Cube("/World/Box", sizes=0.3, positions=[[0.0, 0.0, 2.0]], colors=[0.72, 0.52, 0.30])
GeomPrim(box.paths, apply_collision_apis=True)  # 상자: 충돌체
box_body = RigidPrim(box.paths)  # 상자: 강체. 이제 중력을 받는다
ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[3.0, -3.0, 2.0], target=[0.0, 0.0, 0.6])

# --- 재생. GUI의 ▶ 버튼과 같다. 재생해야 물리가 돈다.
steps = {"n": 0}  # 지금까지 진행된 물리 스텝 수


def count_step(dt, context):
    """물리 한 스텝이 끝날 때마다 불린다. 시뮬레이션 시각 = 스텝 수 × 0.005 s"""
    steps["n"] += 1


callback_id = SimulationManager.register_callback(count_step, IsaacEvents.POST_PHYSICS_STEP)
timeline = omni.timeline.get_timeline_interface()
timeline.play()
frame = 0
while simulation_app.is_running():
    simulation_app.update()  # 한 프레임 = 물리 여러 스텝 + 화면 1장 (몇 스텝인지는 steps로 본다)
    frame += 1
    z = float(box_body.get_world_poses()[0].numpy()[0][2])  # [m] 상자 중심 높이
    print(f"Z {steps['n'] * 0.005:.3f} {z:.4f} steps {steps['n']}", flush=True)  # 시각, 높이, 누적 스텝
    if args.shot and frame == max(args.frames - 5, 1):
        capture_viewport_to_file(get_active_viewport(), args.shot)
    if args.frames and frame >= args.frames:
        break
print("STEP02 OK", flush=True)
timeline.stop()
SimulationManager.deregister_callback(callback_id)
simulation_app.close()
