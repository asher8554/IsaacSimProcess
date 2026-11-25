# (3단계) 창고·컨베이어·팔레트·박스로 공장 라인 장면을 짜고 설비 높이를 실측하는 스크립트
import argparse
import math

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="창 없이 실행")
parser.add_argument("--frames", type=int, default=0, help="이 프레임 수만큼 돌고 끝낸다. 0이면 창을 닫을 때까지")
parser.add_argument("--shot", default="", help="끝나기 직전 화면을 저장할 PNG 경로")
args = parser.parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.timeline
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.rendering_manager import RenderingManager, ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Usd, UsdGeom

stage_utils.create_new_stage()
stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
SimulationManager.set_physics_sim_device("cpu")
SimulationManager.set_physics_dt(1 / 200)
RenderingManager.set_dt(8 / 200)  # [s] 화면 간격 요청값 (2단계에서 잰 실제는 update 1번에 물리 6스텝)
simulation_app.update()

# --- 이번 단계에서 추가: 모델 파일을 장면에 끼워 넣는 add()
assets = get_assets_root_path()  # NVIDIA 에셋 서버 주소. 첫 실행은 내려받느라 오래 걸린다
if assets is None:
    raise RuntimeError("에셋 서버에 연결할 수 없다. 인터넷 연결을 확인한다.")


def add(url, prim_path, position=(0.0, 0.0, 0.0), yaw=0.0):
    """USD 파일 하나를 참조로 불러와 [m] position, [rad] yaw(위에서 본 반시계 회전)에 놓는다.
    url이 "/Isaac/..."면 에셋 서버 파일, "E:/models/my_robot.usd"처럼 드라이브 문자로 시작하면 내 PC 파일이다."""
    stage_utils.add_reference_to_stage(url if url[1:2] == ":" else assets + url, prim_path)
    XformPrim(prim_path, positions=list(position), orientations=[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)], reset_xform_op_properties=True)


add("/Isaac/Environments/Simple_Warehouse/warehouse.usd", "/World/Warehouse")  # 건물·바닥·조명
for i in range(3):  # 2 m짜리 컨베이어 3개를 이어 6 m 라인을 만든다
    add("/Isaac/Props/Conveyors/ConveyorBelt_A05.usd", f"/World/Conveyor_{i}", (-4.0 + 2.0 * i, 0.0, 0.0))
add("/Isaac/Props/Pallet/pallet.usd", "/World/PalletOut", (3.2, -1.5, 0.0))
add("/Isaac/Props/Pallet/pallet.usd", "/World/PalletBuffer", (-3.0, -1.5, 0.0))
# 박스 6개. 파일이 아니라 코드로 만든 정육면체. 물리가 없어 위치를 코드가 정한다 (위치는 나중 단계에서 움직인다)
Cube([f"/World/Boxes/box_{i}" for i in range(6)], sizes=0.35, colors=[0.72, 0.52, 0.30],
     positions=[[-3.7 + 1.08 * i, 0.0, 0.77 + 0.175] for i in range(6)])

while stage_utils.is_stage_loading():  # 서버에서 받는 파일을 다 읽을 때까지 기다린다
    simulation_app.update()
ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[8.5, -8.0, 6.0], target=[0.0, 0.5, 0.8])

# --- 실측. 불러온 모델의 크기는 파일마다 다르다. 감싸는 상자(바운딩 박스)로 높이를 잰다.
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
for path in ("/World/Conveyor_1", "/World/PalletOut"):
    box = cache.ComputeWorldBound(stage_utils.get_current_stage().GetPrimAtPath(path)).ComputeAlignedRange()
    low, high = box.GetMin(), box.GetMax()
    print(f"BBOX {path} x {low[0]:.3f}~{high[0]:.3f}  y {low[1]:.3f}~{high[1]:.3f}  top z {high[2]:.3f}", flush=True)

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
print("STEP03 OK", flush=True)
timeline.stop()
simulation_app.close()
